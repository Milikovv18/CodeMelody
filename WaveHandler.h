#pragma once

#include <vector>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <complex>
#include <valarray>


class WaveHandler
{
public:
	WaveHandler(std::wstring sOutputDevice,
		unsigned int nSampleRate = 44100,
		unsigned int nChannels = 1,
		unsigned int bitsPerSample = 8) :

		m_sampleRate(nSampleRate),
		m_channels(nChannels),
		m_waveHeader{},
		m_userFunction(nullptr)
	{
		// Validate device
		std::vector<std::wstring> devices = Enumerate();
		auto d = std::find(devices.begin(), devices.end(), sOutputDevice);
		if (d != devices.end())
		{
			// Device is available
			int nDeviceID = distance(devices.begin(), d);
			WAVEFORMATEX waveFormat;
			waveFormat.wFormatTag = WAVE_FORMAT_PCM;
			waveFormat.nSamplesPerSec = m_sampleRate;
			waveFormat.wBitsPerSample = bitsPerSample;
			waveFormat.nChannels = m_channels;
			waveFormat.nBlockAlign = (waveFormat.wBitsPerSample / 8) * waveFormat.nChannels;
			waveFormat.nAvgBytesPerSec = waveFormat.nSamplesPerSec * waveFormat.nBlockAlign;
			waveFormat.cbSize = 0;

			m_done = CreateEvent(0, false, false, 0);

			// Open Device if valid
			if (waveOutOpen(&m_hwDevice, nDeviceID, &waveFormat, (DWORD_PTR)m_done, (DWORD_PTR)this, CALLBACK_EVENT) != S_OK)
			{
				throw "Invalid device";
				return;
			}
		}

		// Allocate Wave|Block Memory
		for (int i(0); i < 2; ++i)
		{
			m_waveHeader[i].lpData = new char[2 * FRAME_SIZE];
			m_waveHeader[i].dwBufferLength = 2 * FRAME_SIZE;
			if (!m_waveHeader[i].lpData)
			{
				throw "No memory allocated";
				return;
			}
			ZeroMemory(m_waveHeader[i].lpData, 2 * FRAME_SIZE);

			waveOutPrepareHeader(m_hwDevice, &m_waveHeader[i], sizeof(WAVEHDR));
			waveOutWrite(m_hwDevice, &m_waveHeader[i], sizeof(WAVEHDR));
		}

		fftArray.resize(FOURIER_SAMPLES);
		
		// Generating hamming window
		for (int i(0); i < FOURIER_SAMPLES; i++)
			m_fftWindow[i] = 0.54 - 0.46 * cos(2.0 * M_PI * i / (double)FOURIER_SAMPLES);

		m_thread = std::thread(&WaveHandler::MainThread, this);
	}

	~WaveHandler()
	{
		m_thread.join();
	}

	double GetTime()
	{
		return m_globalTime;
	}


	static std::vector<std::wstring> Enumerate()
	{
		int nDeviceCount = waveOutGetNumDevs();
		std::vector<std::wstring> sDevices;
		WAVEOUTCAPS woc;
		for (int n = 0; n < nDeviceCount; n++)
			if (waveOutGetDevCaps(n, &woc, sizeof(WAVEOUTCAPS)) == S_OK)
				sDevices.push_back(woc.szPname);
		return sDevices;
	}


	/* r=log2(N) */
	void computeFFT(std::valarray<std::complex<double>>& sample)
	{
		unsigned int N = sample.size(), k = N, n;
		double thetaT = 3.14159265358979323846264338328L / N;
		std::complex<double> phiT(cos(thetaT), -sin(thetaT)), T;

		while (k > 1)
		{
			n = k;
			k >>= 1;
			phiT = phiT * phiT;
			T = 1.0L;
			for (unsigned int l = 0; l < k; l++)
			{
				for (unsigned int a = l; a < N; a += n)
				{
					unsigned int b = a + k;
					std::complex<double> t = sample[a] - sample[b];
					sample[a] += sample[b];
					sample[b] = t * T;
				}
				T *= phiT;
			}
		}
		// Decimate
		unsigned int m = (unsigned int)log2(N);
		for (unsigned int a = 0; a < N; a++)
		{
			unsigned int b = a;
			// Reverse bits
			b = (((b & 0xaaaaaaaa) >> 1) | ((b & 0x55555555) << 1));
			b = (((b & 0xcccccccc) >> 2) | ((b & 0x33333333) << 2));
			b = (((b & 0xf0f0f0f0) >> 4) | ((b & 0x0f0f0f0f) << 4));
			b = (((b & 0xff00ff00) >> 8) | ((b & 0x00ff00ff) << 8));
			b = ((b >> 16) | (b << 16)) >> (32 - m);
			if (b > a)
			{
				std::complex t = sample[a];
				sample[a] = sample[b];
				sample[b] = t;
			}
		}
	}

	const int16_t* getFFT()
	{
		// Shifting array if multiple and averaging
		for (int i(0); i < FOURIER_BUFFERS - 1; ++i)
		{
			memcpy(m_fftRes[i], m_fftRes[i + 1], (FOURIER_SAMPLES / 2) * sizeof(int16_t));
			for (int sampleNo(0); sampleNo < FOURIER_SAMPLES / 2; ++sampleNo)
				m_fftRes[FOURIER_BUFFERS - 1][sampleNo] += m_fftRes[i][sampleNo];
		}
		for (int sampleNo(0); sampleNo < FOURIER_SAMPLES / 2; ++sampleNo)
			m_fftRes[FOURIER_BUFFERS - 1][sampleNo] /= FOURIER_BUFFERS;

		// Writing fresh new fft
		for (int i(0); i < FOURIER_SAMPLES / 2; ++i)
			m_fftRes[FOURIER_BUFFERS - 1][i] = 0.3 * m_fftRes[FOURIER_BUFFERS - 1][i] + 0.7 * abs(int16_t(abs(fftArray[i])));

		return m_fftRes[FOURIER_BUFFERS - 1];
	}

	void SetUserFunction(void (*func)(int16_t& data, double time))
	{
		m_userFunction = func;
	}

	int16_t clip(int16_t dSample, int16_t dMax)
	{
		if (dSample >= 0.0)
			return fmin(dSample, dMax);
		else
			return fmax(dSample, -dMax);
	}


	static constexpr size_t FRAME_SIZE = 8'000; // 16 bit samples
	static constexpr size_t FOURIER_BUFFERS = 3; // FFT multi-buffering
	static constexpr size_t FOURIER_SAMPLES = 64;
private:
	void (*m_userFunction)(int16_t& data, double time);

	unsigned int m_sampleRate;
	unsigned int m_channels;
	unsigned int m_blockSamples;

	HANDLE m_done;
	WAVEHDR m_waveHeader[2]{}; // Multi-buffering
	HWAVEOUT m_hwDevice;

	std::thread m_thread;
	std::atomic<double> m_globalTime;

	double m_fftWindow[FOURIER_SAMPLES]{};
	int16_t m_fftRes[FOURIER_BUFFERS][FOURIER_SAMPLES / 2]{}; // minus DC
	std::valarray<std::complex<double>> fftArray;


	void MainThread()
	{
		m_globalTime = 0.0;
		double dTimeStep = 1.0 / (double)m_sampleRate;

		while (true)
		{
			WaitForSingleObject(m_done, INFINITE);
			ResetEvent(m_done);

			for (int bufId(0); bufId < 2; ++bufId)
			{
				if (m_waveHeader[bufId].dwFlags & WHDR_DONE)
				{
					for (int blockPos(0); blockPos < FRAME_SIZE; ++blockPos)
					{
						int16_t& sample = *(int16_t*)&m_waveHeader[bufId].lpData[2 * blockPos];
						m_userFunction(sample, m_globalTime);
						sample = clip(sample, 32767);

						size_t idx = FRAME_SIZE - FOURIER_SAMPLES * 2;
						if (blockPos >= idx && blockPos % 2 == 0)
							fftArray[(blockPos - idx) / 2] =
								std::complex<double>(sample * m_fftWindow[(blockPos - idx) / 2], 0);

						m_globalTime = m_globalTime + dTimeStep;
					}

					computeFFT(fftArray);
					waveOutWrite(m_hwDevice, &m_waveHeader[bufId], sizeof(WAVEHDR));
				}
			}
		}
	}
};