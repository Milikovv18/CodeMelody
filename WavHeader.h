#pragma once

#include <fstream>


class WavFileContent
{
public:
	WavFileContent()
	{
		// Плохое использование типов
		if (sizeof(unsigned long) != 4 || sizeof(unsigned short) != 2)
			throw "Bad CPU";
	}

	void setFile(std::ifstream& file)
	{
		file.read((char*)&chunkId, 4);
		file.read((char*)&chunkSize, 4);
		file.read((char*)&format, 4);
		file.read((char*)&subchunk1Id, 4);
		file.read((char*)&subchunk1Size, 4);
		file.read((char*)&audioFormat, 2);
		file.read((char*)&numChannel, 2);
		file.read((char*)&sampleRate, 4);
		file.read((char*)&byteRate, 4);
		file.read((char*)&blockAlign, 2);
		file.read((char*)&bitsPerSample, 2);
		file.read((char*)&subchunk2Id, 4);
		file.read((char*)&subchunk2Size, 4);

		data = new int16_t[(chunkSize - 36) / 2]{}; // 2 times larger than char
		file.read((char*)data, chunkSize - 36);
	}

	int16_t* getData() { return data; }
	size_t getDataSize() { return (chunkSize - 36) / 2; }
	int getDuration()
	{
		int numSamples = (chunkSize - 36) / (numChannel * (bitsPerSample / 8));
		int durationSeconds = numSamples / sampleRate;
		return durationSeconds;
	}

	~WavFileContent()
	{
		delete[] data;
	}

private:

	unsigned short extra{};
	char chunkId[4]{};
	unsigned long chunkSize{};
	char format[4]{};
	char subchunk1Id[4]{};
	unsigned long subchunk1Size{};
	unsigned short audioFormat{};
	unsigned short numChannel{};
	unsigned long sampleRate{};
	unsigned long byteRate{};
	unsigned short blockAlign{};
	unsigned short bitsPerSample{};
	char subchunk2Id[4]{};
	unsigned long subchunk2Size{};

	int16_t* data = nullptr;
};