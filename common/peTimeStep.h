#pragma once

#include"peSettings.h"

struct TimeStep
{
	float32 dt;
	float32 ratio;
	int32 velocityIteration;
	int32 positionIteration;
};
