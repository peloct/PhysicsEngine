#pragma once

#include"../common/peSettings.h"

const int32 chunkSize = 16 * 1024;
const int32 chunkRefIncrement = 128;
const int32 blockSizeCount = 14;
const int32 maxBlockSize = 640;

struct ChunkRef;
struct Block;
class Profile;

class BlockAllocator
{
public:
	BlockAllocator();
	~BlockAllocator();
	void* allocate(int32 size);
	void free(void* mem, int32 size);

	void getProfile(Profile* profile);

private:
	Block* freeBlocks[blockSizeCount];
	ChunkRef* chunkRefs;
	int32 chunkCapacity;
	int32 chunkCount;

	static int32 blockSizes[blockSizeCount];
	static int32 blockSizeLookUp[maxBlockSize + 1];
	static bool isBlockSizeLookUpInitialized;
};