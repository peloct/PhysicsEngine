#include"peTimer.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

float64 Timer::invFrequency = 0.0f;

Timer::Timer()
{
	LARGE_INTEGER largeInteger;

	if (invFrequency == 0.0f)
	{
		QueryPerformanceFrequency(&largeInteger);
		invFrequency = float64(largeInteger.QuadPart);
		if (invFrequency > 0.0f)
		{
			invFrequency = 1000.0f / invFrequency;
		}
	}

	QueryPerformanceCounter(&largeInteger);
	m_start = float64(largeInteger.QuadPart);
}

void Timer::reset()
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter(&largeInteger);
	m_start = float64(largeInteger.QuadPart);
}

float32 Timer::getMilliseconds() const
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter(&largeInteger);
	float64 count = float64(largeInteger.QuadPart);
	float32 ms = float32(invFrequency * (count - m_start));
	return ms;
}