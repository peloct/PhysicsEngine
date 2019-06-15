#include"peBox.h"

Box::Type Box::getType() const
{
	return Box::Type::box;
}

void Box::computeMassData(MassData* massData, float32 density) const
{
	massData->centerOfMass.set(0.0f, 0.0f, 0.0f);
	massData->mass = density * x * y * z;

	float32 val = massData->mass / 12.0f;
	massData->inertiaTensor.setZero();
	massData->inertiaTensor.m00 = (y * y + z * z) * val;
	massData->inertiaTensor.m11 = (x * x + z * z) * val;
	massData->inertiaTensor.m22 = (x * x + y * y) * val;
}

void Box::computeAABB(AABB* aabb, const Vector3& position, const Matrix3x3& orientationMatrix) const
{
	aabb->maxPos.setZero();

	for (int i = 0; i < 8; ++i)
	{
		Vector3 woldPos = orientationMatrix * vertices[i];
		if (aabb->maxPos.x < woldPos.x)
			aabb->maxPos.x = woldPos.x;
		if (aabb->maxPos.y < woldPos.y)
			aabb->maxPos.y = woldPos.y;
		if (aabb->maxPos.z < woldPos.z)
			aabb->maxPos.z = woldPos.z;
	}

	aabb->minPos = -aabb->maxPos;

	aabb->maxPos += position;
	aabb->minPos += position;
}