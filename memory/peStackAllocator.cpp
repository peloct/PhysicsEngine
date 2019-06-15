#include"peStackAllocator.h"

StackAllocator::StackAllocator() : memory(), stackEntries(), entryCount(0), usedStackSize(0)
{
}

StackAllocator::~StackAllocator()
{
	for (int i = 0; i < entryCount; ++i)
		if (stackEntries[i].isMallocUsed)
			peFree(stackEntries[i].mem);
}

void* StackAllocator::allocate(int32 size)
{
	StackEntry* entryToUse = stackEntries + entryCount;

	if (usedStackSize + size > stackSize)
	{
		entryToUse->isMallocUsed = true;
		entryToUse->mem = peAlloc(size);
	}
	else
	{
		entryToUse->isMallocUsed = false;
		entryToUse->mem = memory + usedStackSize;
		usedStackSize += size;
	}

	entryToUse->size = size;
	++entryCount;

	return entryToUse->mem;
}

void StackAllocator::free(void* mem)
{
	assert(entryCount > 0);
	StackEntry* entryToFree = stackEntries + entryCount - 1;
	assert(entryToFree->mem == mem);

	if (entryToFree->isMallocUsed)
		peFree(entryToFree->mem);
	else
		usedStackSize -= entryToFree->size;

	entryToFree->mem = nullptr;
	--entryCount;
}