#include <iostream>
#include <cstdint>

#include <signal.h> //  our new library 
volatile sig_atomic_t exit_flag = 0;
void exit_flag_function(int sig){
	exit_flag = 1;
}

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>
#include <common/alhelpers.h>

struct Recorder {
	ALCdevice *mDevice;

	long mDataSizeOffset;
	ALuint mDataSize;

	ALuint mChannels;
	ALuint mBits;
	ALuint mSampleRate;
	ALuint mFrameSize;
	ALbyte *mBuffer;
	ALsizei mBufferSize;
	ALenum format;
	ALCenum err;
};

int main()
{
	signal(SIGINT, exit_flag_function);


	Recorder recorder;
	recorder.mDevice = NULL;
	recorder.mDataSizeOffset = 0;
	recorder.mDataSize = 0;
	recorder.mChannels = 1;
	recorder.mBits = 16;
	recorder.mSampleRate = 44100;
	recorder.mFrameSize = recorder.mChannels * recorder.mBits / 8;
	recorder.mBuffer = NULL;
	recorder.mBufferSize = 0;
	recorder.mFrameSize = recorder.mChannels * recorder.mBits / 8;
	recorder.format = AL_NONE;
	recorder.err = ALC_NO_ERROR;
	if(recorder.mChannels == 1)
	{
		if(recorder.mBits == 8)
			recorder.format = AL_FORMAT_MONO8;
		else if(recorder.mBits == 16)
			recorder.format = AL_FORMAT_MONO16;
		else if(recorder.mBits == 32)
			recorder.format = AL_FORMAT_MONO_FLOAT32;
	}
	else if(recorder.mChannels == 2)
	{
		if(recorder.mBits == 8)
			recorder.format = AL_FORMAT_STEREO8;
		else if(recorder.mBits == 16)
			recorder.format = AL_FORMAT_STEREO16;
		else if(recorder.mBits == 32)
			recorder.format = AL_FORMAT_STEREO_FLOAT32;
	}

	recorder.mDevice = alcCaptureOpenDevice(NULL, recorder.mSampleRate, recorder.format, 1024);
	if(!recorder.mDevice)
	{
		std::cout << "Failed to open microphone" << std::endl;
		return 1;
	}

	alcCaptureStart(recorder.mDevice);
	while(!exit_flag)
	{
		recorder.err=alcGetError(recorder.mDevice);
		if (recorder.err != ALC_NO_ERROR) {
			std::cout << "Error" << recorder.err << std::endl;
			break;
		}
		ALCint count = 0;
		alcGetIntegerv(recorder.mDevice, ALC_CAPTURE_SAMPLES, 1, &count);
		if(count < 1)
		{
			al_nssleep(10000000);
			continue;
		}
		if(count > recorder.mBufferSize)
		{
			ALbyte *data = (ALbyte*)calloc(recorder.mFrameSize, (ALuint)count);
			free(recorder.mBuffer);
			recorder.mBuffer = data;
			recorder.mBufferSize = count;
		}
		alcCaptureSamples(recorder.mDevice, recorder.mBuffer, count);

		// plot
		const uint16_t width  = 80;
		const uint16_t height = 20;
		std::cout << count << std::endl;
		for(size_t y = 0; y < height; y++) {
			for(size_t x = 0; x < width; x++) {
				float value = (float)((int16_t*)recorder.mBuffer)[x * count / width] / (INT16_MAX / 10.0f);
				value = (value * 0.5f + 0.5f);
				value *= (float)height;
				std::cout << ((height - 1 - value) < y ? "." : " ");
			}
			std::cout << std::endl;
		}
	}
	alcCaptureStop(recorder.mDevice);
	alcCaptureCloseDevice(recorder.mDevice);
	recorder.mDevice = NULL;
	free(recorder.mBuffer);
	recorder.mBuffer = NULL;
	recorder.mBufferSize = 0;
	std::cout << "Close" << recorder.err << std::endl;
	return 0;
}