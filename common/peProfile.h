#pragma once

#include"peSettings.h"

struct Profile
{
	float32 step;
	float32 checkCollision;
	float32 solvingVC;
	float32 solvingPC;
	float32 syncFixture;
	float32 updateContacts;
	int32 solveCount;
	int32 hitCount;

	int32 blockChunkCount;
	int32 freeBlockCounts[14];
	int32 blockCounts[14];

	void reset()
	{
		step = 0;
		checkCollision = 0;
		solvingVC = 0;
		solvingPC = 0;
		syncFixture = 0;
		updateContacts = 0;
		solveCount = 0;
		hitCount = 0;

		for (int i = 0; i < 14; ++i)
		{
			freeBlockCounts[i] = 0;
			blockCounts[i] = 0;
		}
	}
};