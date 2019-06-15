#pragma once
#include"peSettings.h"
#include<memory.h>

template<typename T, int32 N>
class GrowableStack
{
public:
	GrowableStack()
	{
		size = 0;
		capacity = N;
		stack = staticBuffer;
	}

	~GrowableStack()
	{
		if (stack != staticBuffer)
			peFree(stack);
	}

	void push(const T& val)
	{
		if (size == capacity)
		{
			capacity *= 2;
			T* newBuffer = (T*)peAlloc(capacity * sizeof(T));
			memcpy(newBuffer, stack, size * sizeof(T));
			
			if (stack != staticBuffer)
				peFree(stack);
			stack = newBuffer;
		}

		stack[size] = val;
		++size;
	}

	T pop()
	{
		assert(size > 0);
		--size;
		return stack[size];
	}

	void clear()
	{
		size = 0;
	}

	bool empty()
	{
		return size == 0;
	}

private:
	int32 size;
	int32 capacity;

	T* stack;
	T staticBuffer[N];
};