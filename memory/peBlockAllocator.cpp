#include"peBlockAllocator.h"
#include"../common/peProfile.h"
#include<memory.h>

struct ChunkRef
{
	int32 size;
	Block* pointer; // 개별 청크는 block 들로 쪼개어진다.
};

struct Block
{
	Block* next;
};

// box2d 참조
int32 BlockAllocator::blockSizes[blockSizeCount] =
{
	16,		// 0
	32,		// 1
	64,		// 2
	96,		// 3
	128,	// 4
	160,	// 5
	192,	// 6
	224,	// 7
	256,	// 8
	320,	// 9
	384,	// 10
	448,	// 11
	512,	// 12
	640,	// 13
};

int32 BlockAllocator::blockSizeLookUp[maxBlockSize + 1];
bool BlockAllocator::isBlockSizeLookUpInitialized = false;

BlockAllocator::BlockAllocator()
{
	if (isBlockSizeLookUpInitialized == false)
	{
		isBlockSizeLookUpInitialized = true;
		int index = 0;
		for (int i = 1; i <= maxBlockSize; ++i)
		{
			if (blockSizes[index] < i)
				++index;
			blockSizeLookUp[i] = index;
		}
	}

	chunkCount = 0;
	chunkCapacity = chunkRefIncrement;
	chunkRefs = (ChunkRef*)peAlloc(chunkCapacity * sizeof(ChunkRef));
	memset(chunkRefs, 0, chunkCapacity * sizeof(ChunkRef));
}

BlockAllocator::~BlockAllocator()
{
	for (int i = 0; i < chunkCount; ++i)
		peFree(chunkRefs[i].pointer);
	peFree(chunkRefs);
}

void* BlockAllocator::allocate(int32 size)
{
	assert(size > 0);
	if (size > maxBlockSize)
		return peAlloc(size);

	int32 blockSizeIndex = blockSizeLookUp[size];
	
	if (freeBlocks[blockSizeIndex] == nullptr)
	{
		if (chunkCount == chunkCapacity)
		{
			int oldChunkCapcity = chunkCapacity;
			chunkCapacity += chunkRefIncrement;
			ChunkRef* newBuffer = (ChunkRef*)peAlloc(chunkCapacity * sizeof(ChunkRef));
			memset(newBuffer, 0, chunkCapacity * sizeof(ChunkRef));
			memcpy(newBuffer, chunkRefs, oldChunkCapcity * sizeof(ChunkRef));
			peFree(chunkRefs);
			chunkRefs = newBuffer;
		}

		ChunkRef* chunkToUse = chunkRefs + chunkCount;
		int32 blockSize = blockSizes[blockSizeIndex];
		int32 blockCount = chunkSize / blockSize;

		chunkToUse->size = blockSize;
		chunkToUse->pointer = (Block*)peAlloc(chunkSize);

		for (int i = 0; i < blockCount - 1; ++i)
		{
			Block* eachBlock = (Block*)((int8*)chunkToUse->pointer + blockSize * i);
			eachBlock->next = (Block*)((int8*)chunkToUse->pointer + blockSize * (i + 1));
		}

		Block* lastBlock = (Block*)((int8*)chunkToUse->pointer + blockSize * (blockCount - 1));
		lastBlock->next = nullptr;
		freeBlocks[blockSizeIndex] = chunkToUse->pointer->next;
		++chunkCount;
		return chunkToUse->pointer;
	}
	else
	{
		Block* blockToUse = freeBlocks[blockSizeIndex];
		Block* next = blockToUse->next;
		freeBlocks[blockSizeIndex] = next;
		return (void*)blockToUse;
	}
}

void BlockAllocator::free(void* mem, int32 size)
{
	assert(size > 0);
	if (size > maxBlockSize)
	{
		peFree(mem);
		return;
	}

	int32 blockSizeIndex = blockSizeLookUp[size];
	Block* block = (Block*)mem;
	block->next = freeBlocks[blockSizeIndex];
	freeBlocks[blockSizeIndex] = block;
}

void BlockAllocator::getProfile(Profile* profile)
{
	profile->blockChunkCount = chunkCount;
	for (int i = 0; i < blockSizeCount; ++i)
	{
		Block* block = freeBlocks[i];
		int count = 0;
		while (block)
		{
			++count;
			block = block->next;
		}

		profile->freeBlockCounts[i] = count;
	}

	for (int i = 0; i < chunkCount; ++i)
	{
		int32 blockSizeIndex = blockSizeLookUp[chunkRefs[i].size];
		int32 blockAllocCount = chunkSize / chunkRefs[i].size;
		profile->blockCounts[blockSizeIndex] += blockAllocCount;
	}
}