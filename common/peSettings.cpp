#include"peSettings.h"
#include<stdlib.h>

void* peAlloc(int32 size)
{
	return malloc(size);
}

void peFree(void* mem)
{
	free(mem);
}