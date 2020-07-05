#include <iostream>
#include <string>
#include <cstdint>
#include <vector>
#include <cmath>
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
			for(size_t y = 0; y < height; y++) {
				size_t index = (size * x) / width;   // = size * (x / width) < size
				float value = wave[index];
				value = (float)height * (value - min) / (max - min);
				map_[x][y] = (height - 1 - value) < y;
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
	if (recorder.init_device()) return 1;

	Canvas canvas;

	Fourier fourier(
		recorder.get_buffer_size(),
		recorder.get_sample_rate(),
		canvas.get_width() * 2,
		0.0f, 400.0f
	);

	while(true)
	{
		// get audio
		if (recorder.get_next()) return 1;
		recorder.convert_float();

		// transform fourier
		fourier.transform(recorder.get_buffer_f());

		// plot
		canvas.draw(fourier.get_output(), 0.0f, 0.03f);
		clear();
		canvas.preview();
	}
	return 0;
}