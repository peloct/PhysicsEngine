#pragma once

#include"peSettings.h"

struct TimeStep
{
	float32 dt;
	int32 velConstraintIterCnt;
	int32 posConstraintIterCnt;
};
