// CodeMelody.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <string>

#include "WaveHandler.h"
#include "WavHeader.h"


// Global
WavFileContent fileContent;
WaveHandler wh(WaveHandler::Enumerate()[0], 44100, 2, 16);


namespace console_helper
{
	bool inited(false);

	short leftAlign;
	short centerAlign;
	short rightAlign;

	HANDLE hConsole;


	void init()
	{
		hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

		CONSOLE_SCREEN_BUFFER_INFO csbi;
		GetConsoleScreenBufferInfo(hConsole, &csbi);

		leftAlign = 0;
		centerAlign = csbi.dwSize.X / 2;
		rightAlign = csbi.dwSize.X;
	}

	void centerPrint(std::wstring text, short height)
	{
		if (centerAlign == -1)
			throw "No aligns available";

		COORD printPos{ centerAlign - (text.size() + 1) / 2, height };
		SetConsoleCursorPosition(hConsole, printPos);
		std::wcout << text;
	}

	void leftPrint(std::wstring text, short height)
	{
		if (leftAlign == -1)
			throw "No aligns available";

		COORD printPos{ leftAlign, height };
		SetConsoleCursorPosition(hConsole, printPos);
		std::wcout << text;
	}

	void rightPrint(std::wstring text, short height)
	{
		if (centerAlign == -1)
			throw "No aligns available";

		COORD printPos{ rightAlign - (text.size() + 1), height };
		SetConsoleCursorPosition(hConsole, printPos);
		std::wcout << text;
	}

	void setColor(short color)
	{
		SetConsoleTextAttribute(hConsole, color);
	}
	void unsetColor()
	{
		SetConsoleTextAttribute(hConsole, 7);
	}

	void showCursor(bool show)
	{
		CONSOLE_CURSOR_INFO cursorInfo;
		GetConsoleCursorInfo(hConsole, &cursorInfo);
		cursorInfo.bVisible = show;
		SetConsoleCursorInfo(hConsole, &cursorInfo);
	}
}


namespace sound_gen
{
	short direction(1);
	short isEnabled(0);

	bool isPaused(false);
	bool isWigglingAllowed(false);

	int64_t skipSize(0);
	std::mutex skipMutex;


	void someCoolFunc(int16_t& data, double time)
	{
		static double lastTime(0.0);
		static const int16_t* fileData = fileContent.getData();
		static const int64_t maxPos = fileContent.getDataSize();
		static const int duration = fileContent.getDuration();
		static const double customWavesFactor(0.5);
		static int64_t pos(0);

		double resNote(0.0);
		unsigned int customWavesCount(0);

		// Custom notes
		for (int noteNo(0); noteNo < 9; ++noteNo)
		{
			if (isEnabled & (short)pow(2, noteNo)) {
				resNote += sin(400.0 * (noteNo + 1.0) * time);
				customWavesCount++;
			}
		}

		// Wiggling
		double ampWave(1.0);
		if (1.0 - sin(10.0 * time) < 0.01) {
			isWigglingAllowed = true;
		}
		else {
			isWigglingAllowed = false;
		}

		if (isEnabled & 512)
			ampWave = sin(10.0 * time);

		// Compressing custom waves
		if (customWavesCount > 0)
			resNote /= customWavesCount; // Custom waves treated equally

		// Combining custom and file waves
		resNote = customWavesFactor * resNote;
		if (!isPaused)
			resNote += (1 - customWavesFactor) * (fileData[pos] * 0.000030517578125);

		// Skipping
		skipMutex.lock();
		if (skipSize < 0) {
			skipSize = (-skipSize) > pos ? pos : skipSize;
		}
		else if (skipSize > 0) {
			skipSize = skipSize > maxPos - pos ? maxPos - pos : skipSize;
		}

		// Printing
		std::wstring output;
		if (pos % 10'000 == 0) // Progress bar
		{
			short done = 50 * pos / maxPos;
			short timeSince = duration * pos / maxPos;
			output += std::to_wstring(timeSince / 60) + L":" + std::to_wstring(timeSince % 60);
			output += L" [" + std::wstring(done, L'#');
			output += std::wstring(50ULL - done, L'_');
			output += L"] ";
			output += std::to_wstring(duration / 60) + L":" + std::to_wstring(duration % 60);
			console_helper::centerPrint(output, 22);
		}

		// Drawing spectre
		if (time - lastTime >= 0.2)
		{
			const int16_t* fftArray = wh.getFFT();
			for (int line(0); line < 10; ++line)
			{
				output.clear();
				int16_t validValue = 3'276.7 * (9ULL - line);
				for (int j(0); j < 0.5 * WaveHandler::FOURIER_SAMPLES; ++j)
					output += ((fftArray[j] >= validValue) ? L'#' : L' ');

				if (line < 3)
					console_helper::setColor(FOREGROUND_RED | FOREGROUND_INTENSITY);
				else
					console_helper::setColor(FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY);

				console_helper::centerPrint(output, line + 9);
				console_helper::unsetColor();
			}
			lastTime = time;
		}

		// Summary
		data = (resNote * ampWave) * 32767; // 16 bit
		if (!isPaused && (pos > 0 || direction > 0) && (pos < maxPos || direction < 0))
			pos += direction + skipSize;

		skipSize = 0;
		skipMutex.unlock();
	}
}


int main()
{
	// Should hurry with this, bc WaveHandler is already started (bug???)
	wh.SetUserFunction(sound_gen::someCoolFunc);

	console_helper::init();
	console_helper::showCursor(false);

	console_helper::leftPrint(L"Press ASDFGHJKL to play some notes", 0);
	console_helper::leftPrint(L"Or press Shift to do cool effect with sound", 1);
	console_helper::leftPrint(L"Also you can press space to play/pause", 2);
	console_helper::leftPrint(L"Or up/down arrow to change direction of the song", 3);
	console_helper::leftPrint(L"Finally press left/right arrows to rewind song", 4);
	console_helper::leftPrint(L"If you really want to,", 5);
	console_helper::leftPrint(L"you can also change hardcoded coeficients", 6);

	console_helper::rightPrint(L"What you see on the screen is a...", 0);
	console_helper::rightPrint(L"16 bit wav player + piano maybe?", 1);
	console_helper::rightPrint(L"And even a little of equalizer here", 2);
	console_helper::rightPrint(L"Actually FFT doesnt work properly", 3);
	console_helper::rightPrint(L"especially with handmade sin waves", 4);
	console_helper::rightPrint(L"But dont mind it, anyway its cool", 5);

	// Выбор аудио файла
	console_helper::centerPrint(L"Faded.wav", 20);

	std::ifstream file("Faded.wav", std::ios::binary);
	if (file.fail() || file.eof())
		throw "Cant open wav file";
	fileContent.setFile(file);

	const char keys[]{ "ASDFGHJKL" };
	bool spacePressed(false);
	while (true)
	{
		// Custom notes
		for (int keyNo(0); keyNo < std::size(keys) - 1; ++keyNo)
		{
			if (GetAsyncKeyState(keys[keyNo]) & 0x8000)
				sound_gen::isEnabled |= (short)pow(2, keyNo);
			else
				sound_gen::isEnabled &= ~(short)pow(2, keyNo);
		}

		// Wiggling
		if (sound_gen::isWigglingAllowed && (GetAsyncKeyState(VK_SHIFT) & 0x8000))
			sound_gen::isEnabled |= 512;
		else if (sound_gen::isWigglingAllowed)
			sound_gen::isEnabled &= ~512;

		// Play direction
		if (GetAsyncKeyState(VK_DOWN) & 0x8000)
			sound_gen::direction = -1;
		else if (GetAsyncKeyState(VK_UP) & 0x8000)
			sound_gen::direction = 1;

		// Skipping
		if (GetAsyncKeyState(VK_LEFT) & 0x8000) {
			sound_gen::skipMutex.lock();
			sound_gen::skipSize = -500;
			sound_gen::skipMutex.unlock();
		}
		else if (GetAsyncKeyState(VK_RIGHT) & 0x8000) {
			sound_gen::skipMutex.lock();
			sound_gen::skipSize = 500;
			sound_gen::skipMutex.unlock();
		}

		// Pause
		if (GetAsyncKeyState(VK_SPACE) & 0x8000) {
			if (!spacePressed) {
				spacePressed = true;
				sound_gen::isPaused = !sound_gen::isPaused;
			}
		}
		else {
			spacePressed = false;
		}
	}

	return 0;
}
