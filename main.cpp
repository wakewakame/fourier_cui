#include <iostream>
#include <cstdint>
#include <vector>
#include <cmath>
#include <cassert>

#include <stdlib.h>
#include <signal.h>
#ifdef _WIN32
void clear() {
	system("cls");
}
#else
volatile sig_atomic_t exit_flag = 0;
void exit_flag_function(int sig){
	exit_flag = 1;
}
void clear() {
	system("clear");
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
		buffer_.resize(buffer_size_ * 2);
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
			size_ -= buffer_size_;
			for(size_t i = 0; i < size_; i++) {
				buffer_[i] = buffer_[buffer_size_ + i];
			}
		}
		while(true) {
			ALCint count;
			alcGetIntegerv(mic_device_, ALC_CAPTURE_SAMPLES, 1, &count);
			if(count < 1) {
				al_nssleep(10000000);
				continue;
			}
			while (count > buffer_size_) {
				ALCint size = (count >= 2 * buffer_size_) ? buffer_size_ : count - buffer_size_;
				alcCaptureSamples(mic_device_, &buffer_[index_], size);
				count -= size;
				std::cout << "count : " << count << std::endl;
			}
			alcCaptureSamples(mic_device_, &buffer_[index_], count);
			index_ += count;
			size_ += count;
			assert(index_ < buffer_size_ * 2);
			assert(size_ < buffer_size_ * 2);
			if (index_ >= buffer_size_) {
				index_ -= buffer_size_;
				err_ = alcGetError(mic_device_);
				if (err_ != ALC_NO_ERROR) {
					std::cout << "Error : " << err_ << std::endl;
					return 1;
				}
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
	inline float* get_buffer_f() { return &buffer_f_[0]; }
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
	void transform(const float* const input_buffer) {
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

int main()
{
	signal(SIGINT, exit_flag_function);

	Recorder recorder;
	if (recorder.init_device()) return 1;

	Fourier fourier(
		recorder.get_buffer_size(),
		recorder.get_sample_rate(),
		80, 0.0f, 400.0f
	);

	while(!exit_flag)
	{
		// get audio
		if (recorder.get_next()) return 1;
		recorder.convert_float();

		// transform fourier
		fourier.transform(recorder.get_buffer_f());

		// plot
		const uint16_t width  = 80;
		const uint16_t height = 20;
		auto&& output = fourier.get_output();
		clear();
		for(size_t y = 0; y < height; y++) {
			for(size_t x = 0; x < width; x++) {
				float value = output[x * output.size() / width] * 32.0f;
				//value = (value * 0.5f + 0.5f);
				value *= (float)height;
				std::cout << ((height - 1 - value) < y ? "." : " ");
			}
			std::cout << std::endl;
		}
	}
	
	std::cout << "Close" << std::endl;
	return 0;
}