#pragma once

#include"shape/peShape.h"
#include"common/peMath.h"

class Rigidbody;
class BroadPhase;

struct FixtureDef
{
	FixtureDef()
	{
		shape = nullptr;
		density = 0;
		restitution = 0;
		friction = 0;
	}

	Shape* shape;
	float32 density;
	float32 restitution;
	float32 friction;
};

class Fixture
{
public:
	void computeAABB(AABB* aabb) const;
	void computeMassData(MassData* massData) const;
	void synchronize();
	const Shape* getShape() const { return shape; }
	const Rigidbody* getRigidbody() const { return rigidbody; }

	Vector3 getBodyPosition() const;
	Vector3 getPosition() const;
	Matrix4x4 getBodyLocalToWorld() const;
	Matrix4x4 getLocalToWorld() const;
	Matrix3x3 getOrientation() const;
	float32 getFriction() const { return friction; }
	float32 getRestitution() const { return restitution; }

private:
	friend class ContactManager;
	friend class Rigidbody;

	Fixture(const FixtureDef& def, Rigidbody* rigidbody, BroadPhase* broadPhase);
	~Fixture();
	Fixture() : shape(nullptr), rigidbody(nullptr), broadPhase(nullptr), density(0.0f), restitution(0.0f), friction(0.0f), aabbID(-1) {}

	Shape* shape;
	Rigidbody* rigidbody;
	BroadPhase* broadPhase;
	float32 density;
	float32 restitution;
	float32 friction;
	int32 aabbID;
};