#pragma once

#include<assert.h>

typedef float float32;
typedef double float64;
typedef int int32;
typedef char int8;

#define NULL_ID -1
#define BOUNCE_THRESHOLD -1.0f
#define BAUMGARTE 0.2f
#define LINEAR_SLOP 0.005f
//#define DEBUG_LOG

void* peAlloc(int32 size);
void peFree(void* mem);

void peSetLogLevel(int level);
void peStartLogging();
void peFinishLogging();
void peLog(const char* string, ...);