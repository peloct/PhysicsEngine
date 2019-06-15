#pragma once

#include"../common/peMath.h"

class AABB
{
public:
	Vector3 minPos;
	Vector3 maxPos;

	AABB() {}

	AABB(Vector3 a, Vector3 b)
	{
		minPos.x = peMinf(a.x, b.x);
		minPos.y = peMinf(a.y, b.y);
		minPos.z = peMinf(a.z, b.z);
		maxPos.x = peMaxf(a.x, b.x);
		maxPos.y = peMaxf(a.y, b.y);
		maxPos.z = peMaxf(a.z, b.z);
	}

	void combine(const AABB& aabb)
	{
		minPos.x = peMinf(minPos.x, aabb.minPos.x);
		minPos.y = peMinf(minPos.y, aabb.minPos.y);
		minPos.z = peMinf(minPos.z, aabb.minPos.z);
		maxPos.x = peMaxf(maxPos.x, aabb.maxPos.x);
		maxPos.y = peMaxf(maxPos.y, aabb.maxPos.y);
		maxPos.z = peMaxf(maxPos.z, aabb.maxPos.z);
	}

	void combine(const AABB& a, const AABB& b)
	{
		minPos.x = peMinf(a.minPos.x, b.minPos.x);
		minPos.y = peMinf(a.minPos.y, b.minPos.y);
		minPos.z = peMinf(a.minPos.z, b.minPos.z);
		maxPos.x = peMaxf(a.maxPos.x, b.maxPos.x);
		maxPos.y = peMaxf(a.maxPos.y, b.maxPos.y);
		maxPos.z = peMaxf(a.maxPos.z, b.maxPos.z);
	}

	bool contains(const AABB& aabb) const
	{
		return minPos <= aabb.minPos && aabb.maxPos <= maxPos;
	}

	float32 getVolume() const
	{
		float32 dx = maxPos.x - minPos.x;
		float32 dy = maxPos.y - minPos.y;
		float32 dz = maxPos.z - minPos.z;
		return dx * dy * dz;
	}

	float32 getSurfaceArea() const
	{
		float32 dx = maxPos.x - minPos.x;
		float32 dy = maxPos.y - minPos.y;
		float32 dz = maxPos.z - minPos.z;

		return 2.0f * (dx * dy + dy * dz + dx * dz);
	}

	static bool isOverlapped(const AABB& a, const AABB& b)
	{
		if (a.maxPos.x < b.minPos.x)
			return false;
		if (a.maxPos.y < b.minPos.y)
			return false;
		if (a.maxPos.z < b.minPos.z)
			return false;
		if (a.minPos.x > b.maxPos.x)
			return false;
		if (a.minPos.y > b.maxPos.y)
			return false;
		if (a.minPos.z > b.maxPos.z)
			return false;
		return true;
	}
};