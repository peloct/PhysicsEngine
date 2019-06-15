#include"peFixture.h"
#include"peRigidbody.h"
#include"collision/peBroadPhase.h"

Fixture::Fixture(const FixtureDef& def, Rigidbody* rigidbody, BroadPhase* broadPhase)
{
	assert(def.shape != nullptr && def.density > 0.0f);

	this->rigidbody = rigidbody;
	this->broadPhase = broadPhase;

	shape = def.shape;
	density = def.density;
	restitution = def.restitution;
	friction = def.friction;

	AABB aabb;
	computeAABB(&aabb);
	aabbID = broadPhase->insertAABB(aabb, this);
}

Fixture::~Fixture()
{
	broadPhase->removeAABB(aabbID);
}

void Fixture::computeAABB(AABB* aabb) const
{
	shape->computeAABB(aabb, rigidbody->bodyPosition, rigidbody->orientationMatrix);
}

void Fixture::computeMassData(MassData* massData) const
{
	shape->computeMassData(massData, density);
}

void Fixture::synchronize()
{
	AABB aabb;
	computeAABB(&aabb);
	broadPhase->updateAABB(aabbID, aabb);
}

Vector3 Fixture::getBodyPosition() const
{
	return rigidbody->bodyPosition;
}

Vector3 Fixture::getPosition() const
{
	return rigidbody->massPosition;
}

Matrix4x4 Fixture::getBodyLocalToWorld() const
{
	return Matrix4x4(rigidbody->orientationMatrix, rigidbody->bodyPosition);
}

Matrix4x4 Fixture::getLocalToWorld() const
{
	return Matrix4x4(rigidbody->orientationMatrix, rigidbody->massPosition);
}

Matrix3x3 Fixture::getOrientation() const
{
	return rigidbody->orientationMatrix;
}