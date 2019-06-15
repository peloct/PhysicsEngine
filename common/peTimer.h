#pragma once

#include"peSettings.h"

class Timer
{
public:
	/// Constructor
	Timer();

	/// Reset the timer.
	void reset();

	/// Get the time since construction or the last reset.
	float32 getMilliseconds() const;

private:

	float64 m_start;
	static float64 invFrequency;
};