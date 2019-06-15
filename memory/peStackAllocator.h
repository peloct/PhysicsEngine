#pragma once

#include"../common/peSettings.h"

const int32 stackSize = 100 * 1024;
const int32 maxStackEntries = 32;

struct StackEntry
{
	int32 size;
	void* mem;
	bool isMallocUsed;
};

class StackAllocator
{
public:
	StackAllocator();
	~StackAllocator();

	void* allocate(int32 size);
	void free(void* mem);

private:
	int8 memory[stackSize];
	int32 entryCount;
	int32 usedStackSize;
	StackEntry stackEntries[maxStackEntries];
};