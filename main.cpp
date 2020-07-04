#include <iostream>
#include <cstdint>
#include <vector>

#include <signal.h> //  our new library 
volatile sig_atomic_t exit_flag = 0;
void exit_flag_function(int sig){
	exit_flag = 1;
}

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
			assert(count <= buffer_size_);
			if(count < 1) {
				al_nssleep(10000000);
				continue;
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
};

int main()
{
	signal(SIGINT, exit_flag_function);

	Recorder recorder;
	if (recorder.init_device()) return 1;

	while(!exit_flag)
	{
		if (recorder.get_next()) return 1;

		// plot
		const uint16_t width  = 80;
		const uint16_t height = 20;
		for(size_t y = 0; y < height; y++) {
			for(size_t x = 0; x < width; x++) {
				float value = (float)(recorder.get_buffer()[x * recorder.get_buffer_size() / width]) / (INT16_MAX / 8.0f);
				value = (value * 0.5f + 0.5f);
				value *= (float)height;
				std::cout << ((height - 1 - value) < y ? "." : " ");
			}
			std::cout << std::endl;
		}
	}
	
	std::cout << "Close" << std::endl;
	return 0;
}