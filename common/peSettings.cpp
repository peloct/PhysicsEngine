#include"peSettings.h"
#include<stdlib.h>
#include<stdio.h>
#include<stdarg.h>
#include<stdlib.h>
#include<string>
#include<fstream>
#include<iostream>

void* peAlloc(int32 size)
{
	return malloc(size);
}

void peFree(void* mem)
{
	free(mem);
}

int logLevel = 0;
std::ofstream logFile;
std::ofstream deepLogFile;

void peSetLogLevel(int level)
{
	logLevel = level;
}

void peStartLogging()
{
	logFile = std::ofstream("log.txt");
	deepLogFile = std::ofstream("deepLog.txt");

	if (logFile.is_open() && deepLogFile.is_open())
	{
		std::cout << "succeed to open log" << std::endl;
	}
	else
	{
		std::cout << "failed to open log" << std::endl;
	}
}

void peFinishLogging()
{
	logFile.close();
	deepLogFile.close();
}

void peLog(const char* string, ...)
{
	char buffer[200];

	va_list args;
	va_start(args, string);
	vsprintf_s(buffer, string, args);
	va_end(args);

	std::string str = buffer;

	if (logLevel == 0)
	{
		if (logFile.is_open())
			logFile << str;
	}
	else
	{
		if (deepLogFile.is_open())
			deepLogFile << str;
	}
}