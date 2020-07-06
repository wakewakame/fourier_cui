// ToDo
//   - Recorderのマルチスレッド化
//       alcCaptureSamplesを呼び出す周期が遅すぎるとエラーが出るようなので。
//
//   - ちゃんとした終了処理の実装
//       Ctrl+Cで終了はするが、デストラクタが呼ばれていない気がする。
//
//   - ちゃんとしたエラー処理の実装
//       マイクが見つからなかったときなど

#include <iostream>
#include <string>
#include <cstdint>
#include <vector>
#include <cmath>
#include <utility>
#include <cassert>

#include <cstdlib>
#ifdef _WIN32
void clear() {
	std::system("cls");
}
#else
void clear() {
	std::system("clear");
}
#endif

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>
#include <common/alhelpers.h>

class Recorder {
private:
	ALCdevice *mic_device_;
	ALuint sample_rate_;
	size_t buffer_size_;
	std::vector<int16_t> buffer_;
	std::vector<float> buffer_f_;
	ALCint index_, size_;
	bool is_recording_;
	ALCenum err_;

public:
	Recorder() : mic_device_(nullptr), is_recording_(false), err_(ALC_NO_ERROR) {}
	virtual ~Recorder() { free_device(); }
	int init_device(ALuint sample_rate = 44100, ALuint buffer_size = 1024) {
		free_device();
		sample_rate_ = sample_rate;
		buffer_size_ = buffer_size;
		buffer_.resize(buffer_size_ * 2 - 1);
		buffer_f_.resize(buffer_size_);
		index_ = 0;
		size_ = 0;
		err_ = ALC_NO_ERROR;
		mic_device_ = alcCaptureOpenDevice(NULL, sample_rate_, AL_FORMAT_MONO16, buffer_size_);
		if(!mic_device_)
		{
			std::cout << "Failed to open microphone" << std::endl;
			return 1;
		}
		alcCaptureStart(mic_device_);
		err_ = alcGetError(mic_device_);
		if (err_ != ALC_NO_ERROR) {
			std::cout << "Error : " << err_ << std::endl;
			free_device();
			return 1;
		}
		is_recording_ = true;
		return 0;
	}
	int get_next() {
		if (size_ >= buffer_size_) {
			if (size_ < 2 * buffer_size_) {
				for(size_t i = 0; i < size_ - buffer_size_; i++) {
					buffer_[i] = buffer_[buffer_size_ + i];
				}
				size_ -= buffer_size_;
			}
			else {
				index_ = 0;
				size_ = 0;
			}
		}
		while(true) {
			ALCint count;
			alcGetIntegerv(mic_device_, ALC_CAPTURE_SAMPLES, 1, &count);
			if(count < 1) {
				al_nssleep(10000000);
				continue;
			}
			if (buffer_.size() < index_ + count) {
				buffer_.resize(index_ + count);
			}
			alcCaptureSamples(mic_device_, &buffer_[index_], count);
			err_ = alcGetError(mic_device_);
			if (err_ != ALC_NO_ERROR) {
				std::cout << "Error : " << err_ << std::endl;
				return 1;
			}
			index_ += count;
			size_ += count;
			if (index_ >= buffer_size_) {
				index_ -= buffer_size_;
				return 0;
			}
		}
		return 0;
	}
	void convert_float() {
		for(size_t i = 0; i < buffer_f_.size(); i++) {
			buffer_f_[i] = (float)buffer_[i] / (float)INT16_MAX;
		}
	}
	void free_device() {
		if (!mic_device_) return;
		if (is_recording_) alcCaptureStop(mic_device_);
		alcCaptureCloseDevice(mic_device_);
		mic_device_ = nullptr;
		return;
	}
	inline ALuint get_sample_rate() const { return sample_rate_; }
	inline size_t get_buffer_size() const { return buffer_size_; }
	inline int16_t* get_buffer() { return &buffer_[0]; }
	inline std::vector<float>& get_buffer_f() { return buffer_f_; }
};

class Fourier {
private:
	struct SinCos {
		float sin = 0, cos = 0;
		SinCos operator +=(const SinCos& right_value) {
			this->sin += right_value.sin;
			this->cos += right_value.cos;
			return *this;
		}
	};
	size_t input_size_;
	float sample_rate_;
	std::vector<float> output_buffer_;
	std::vector<std::vector<SinCos>> convolution_map_;
	float min_hz_;
	float max_hz_;
	const float pi = std::acos(-1.0f);

public:
	Fourier(uint16_t input_size = 1024, float sample_rate = 44100.0f, uint16_t output_size = 1024, float min_hz = 1.0f, float max_hz = 1024.0f) :
		sample_rate_(sample_rate), min_hz_(min_hz), max_hz_(max_hz)
	{
		assert(input_size >= 2);
		assert(output_size >= 2);
		input_size_ = input_size;
		output_buffer_.resize(output_size);
		convolution_map_.resize(output_buffer_.size(), std::vector<SinCos>(input_size_));
	}
	virtual ~Fourier() {

	}
	void transform(const std::vector<float>& input_buffer) {
		assert(input_buffer.size() == input_size_);
		for(size_t output_index = 0; output_index < output_buffer_.size(); output_index++) {
			for(size_t input_index = 0; input_index < input_size_; input_index++) {
				float p_input  = (float)input_index / sample_rate_;
				float p_output = (float)output_index / (float)(output_buffer_.size() - 1);
				float hz = (1.0f - p_output) * min_hz_ + p_output * max_hz_;
				float omega = hz * p_input * 2.0f * pi;
				convolution_map_[output_index][input_index].sin = std::sin(omega) * input_buffer[input_index];
				convolution_map_[output_index][input_index].cos = std::cos(omega) * input_buffer[input_index];
			}
		}
		for(size_t output_index = 0; output_index < output_buffer_.size(); output_index++) {
			for(size_t input_index = 1; input_index < input_size_; input_index++) {
				convolution_map_[output_index][0] += convolution_map_[output_index][input_index];
			}
			output_buffer_[output_index] = std::sqrt(
				std::pow(convolution_map_[output_index][0].sin, 2.0f) +
				std::pow(convolution_map_[output_index][0].cos, 2.0f)
			) / (float)input_size_;
		}
	}
	int get_peak_index() {
		size_t max_value_index;
		float max_value;
		for(size_t index = 0; index < output_buffer_.size(); index++) {
			if (output_buffer_[index] > max_value) {
				max_value = output_buffer_[index];
				max_value_index = index;
			}
		}
		if (max_value < 0.001f) return -1;
		return max_value_index;
	}
	float index_to_hz(int index) {
		if (index < 0) return 0.0f;
		float p_output = (float)index / (float)(output_buffer_.size() - 1);
		float hz = (1.0f - p_output) * min_hz_ + p_output * max_hz_;
		return hz;
	}
	inline std::vector<float>& get_output() { return output_buffer_; }
};

class Canvas {
private:
	uint16_t width_, height_;
	std::vector<std::vector<bool>> map_;
	std::string output;
	class Dot2x3 {
	private:
		union {
			uint8_t c_str[4];
			uint32_t number;
		};
	public:
		Dot2x3(uint8_t num = 0) : number((*((uint32_t*)(u8"\u2800"))) + ((uint32_t)num * 0x10000)) {}
		virtual ~Dot2x3() {}
		operator const char* () { return (const char*)c_str; }
	};

public:
	Canvas(uint16_t width = 80, uint16_t height = 20) :
		width_(width), height_(height), map_(width * 2, std::vector<bool>(height * 3, false)) {}
	virtual ~Canvas() {}
	void clear() {
		for(size_t x = 0; x < map_.size(); x++) {
			for(size_t y = 0; y < map_[x].size(); y++) {
				map_[x][y] = false;
			}
		}
	}
	void slide() {
		for(size_t x = 0; x < map_.size(); x++) {
			for(size_t y = 0; y < map_[x].size() - 1; y++) {
				map_[x][y] = map_[x][y + 1];
			}
		}
		for(size_t x = 0; x < map_.size(); x++) {
			map_[x][map_[x].size() - 1] = false;
		}
	}
	void draw(const std::vector<float>& wave, const float min = 0.0f, const float max = 1.0f) {
		draw(&wave[0], wave.size(), min, max);
	}
	void draw(
		const float* wave, const size_t size,
		const float min = 0.0f, const float max = 1.0f
	) {
		const size_t width = map_.size();
		for(size_t x = 0; x < width; x++) {
			const size_t height = map_[x].size();
			size_t index = (size * x) / width;   // = size * (x / width) < size
			float value = wave[index];
			value = (float)height * (value - min) / (max - min);
			line(
				x, height - 1 - (int)value,
				x, height - 1,
				2
			);
		}
	}
	inline void point(int x, int y, int type = 1) {
		if (
			0 <= x && x < map_.size() &&
			0 <= y && y < map_[x].size()
		) {
			if (type == 0) map_[x][y] = false;
			if (type == 1) map_[x][y] = true;
			if (type == 2) map_[x][y] = !map_[x][y];
		}
	}
	inline void line(int x_1, int y_1, int x_2, int y_2, int type = 1) {
		if ((x_1 == x_2) && (y_1 == y_2)) {
			point(x_1, y_1);
			return;
		}
		if (std::abs(x_2 - x_1) > std::abs(y_2 - y_1)) {
			if (x_1 > x_2) {
				std::swap(x_1, x_2);
				std::swap(y_1, y_2);
			}
			for(int x = x_1; x <= x_2; x++) {
				int y = (y_1 * (x_2 - x) + y_2 * (x - x_1)) / (x_2 - x_1);
				point(x, y, type);
			}
		}
		else {
			if (y_1 > y_2) {
				std::swap(x_1, x_2);
				std::swap(y_1, y_2);
			}
			for(int y = y_1; y <= y_2; y++) {
				int x = (x_1 * (y_2 - y) + x_2 * (y - y_1)) / (y_2 - y_1);
				point(x, y, type);
			}
		}
	}
	inline void rect(int x_1, int y_1, int x_2, int y_2, bool fill = false, int type = 1) {
		if (x_1 > x_2) std::swap(x_1, x_2);
		if (y_1 > y_2) std::swap(y_1, y_2);
		if ((x_2 - x_1 <= 2) || (y_2 - y_1 <= 2)) fill = true;
		if (fill) {
			for(int x = x_1; x <= x_2; x++) {
				for(int y = y_1; y <= y_2; y++) {
					point(x, y, type);
				}
			}
		}
		else {
			line(x_1, y_1, x_2, y_1);
			line(x_1, y_2, x_2, y_2);
			line(x_1, y_1 + 1, x_1, y_2 - 1);
			line(x_2, y_1 + 1, x_2, y_2 - 1);
		}
	}
	inline void ellipse(float x, float y, float r, bool fill = false, int type = 1) {
		if (fill) {
			for(int x_i = (int)(x - r); x_i <= (int)(x + r); x_i++) {
				for(int y_i = (int)(y - r); y_i <= (int)(y + r); y_i++) {
					if (pow((float)x_i - x, 2.0f) + pow((float)y_i - y, 2.0f) < pow(r, 2.0f)) {
						point(x_i, y_i, type);
					}
				}
			}
		}
		else {
			for(int i = 0; i < 16; i++) {
				line(
					(int)(x - r * std::cos(2.0f * std::acos(-1.0f) * (float)(i + 0) / 16.0f)),
					(int)(y - r * std::sin(2.0f * std::acos(-1.0f) * (float)(i + 0) / 16.0f)),
					(int)(x - r * std::cos(2.0f * std::acos(-1.0f) * (float)(i + 1) / 16.0f)),
					(int)(y - r * std::sin(2.0f * std::acos(-1.0f) * (float)(i + 1) / 16.0f))
				);
			}
		}
	}
	void preview() {
		output.clear();
		for(uint16_t y = 0; y < height_; y++) {
			for(uint16_t x = 0; x < width_; x++) {
				uint8_t number = 
					map_[x * 2 + 0][y * 3 + 0] * 0b000001 +
					map_[x * 2 + 0][y * 3 + 1] * 0b000010 +
					map_[x * 2 + 0][y * 3 + 2] * 0b000100 +
					map_[x * 2 + 1][y * 3 + 0] * 0b001000 +
					map_[x * 2 + 1][y * 3 + 1] * 0b010000 +
					map_[x * 2 + 1][y * 3 + 2] * 0b100000;
				output += Dot2x3(number);
			}
			if (y != (height_ - 1)) output += "\n";
		}
		std::cout << output << std::endl;
	}
	inline int get_width() const { return width_; }
	inline int get_height() const { return height_; }
};

int main()
{
	Recorder recorder;
	if (recorder.init_device(44100, 1024)) return 1;

	Canvas canvas(80, 20);

	Fourier fourier(
		recorder.get_buffer_size(),
		recorder.get_sample_rate(),
		canvas.get_width() * 2,
		300.0f, 3000.0f
	);

	while(true)
	{
		// get audio
		if (recorder.get_next()) return 1;
		recorder.convert_float();

		// transform fourier
		fourier.transform(recorder.get_buffer_f());
		int peak_index = fourier.get_peak_index();
		float peak_hz = fourier.index_to_hz(peak_index);
		float d_f = 12.0f * std::log2(peak_hz / 440.0f) + 9.0f + 0.5f;
		d_f = d_f - 12.0f * std::floor(d_f / 12.0f);
		int d_i = (int)d_f;
		std::string d_array[] = {
			"ド",
			"ド#",
			"レ",
			"レ#",
			"ミ",
			"ファ",
			"ファ#",
			"ソ",
			"ソ#",
			"ラ",
			"ラ#",
			"シ"
		};
		std::string d_c = (d_i >= 0) ? d_array[d_i] : "";


		// plot
		///*
		//canvas.slide();
		canvas.clear();
		const int x = (int)(20.0f * 7.0f * d_f / 12.0f);
		if (x >= 0) {
			canvas.line(x + 0, 45, x - 2, 55);
			canvas.line(x - 2, 55, x + 2, 55);
			canvas.line(x + 2, 55, x + 0, 45);
		}
		const int w = 20, h = 40;
		for(int i = 0; i < 7; i++) {
			canvas.rect(i * w, 0, (i + 1) * w - 1, h, false, 1);
			if (i != 2 && i != 6) {
				canvas.rect(i * w + w * 3 / 4, 0, (i + 1) * w + w / 4, h * 3 / 5, true, 1);
			}
		}
		//canvas.draw(recorder.get_buffer_f(), -0.1f, 0.1f);
		//canvas.draw(fourier.get_output(), 0.0f, 0.03f);
		//canvas.point(peak_index, 59);
		//canvas.ellipse(20.0f * 7.0f * d_f / 12.0f, 50.0f, 5.0f);

		clear();
		canvas.preview();
		std::cout << d_c << std::endl;
		//*/

		/*
		std::string out = "";
		for(int i = 0; d_i >= 0 && i < d_i - 1; i++) {
			out += "　";
		}
		out += d_c;
		std::cout << out << std::endl;
		*/
	}
	return 0;
}