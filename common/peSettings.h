#pragma once

#include<assert.h>

typedef float float32;
typedef double float64;
typedef int int32;
typedef char int8;

void* peAlloc(int32 size);
void peFree(void* mem);

#define NULL_ID -1