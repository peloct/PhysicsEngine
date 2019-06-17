#pragma once

#include"common/peMath.h"
#include"peFixture.h"

class World;
class Renderer;
class ContactRef;

enum RigidbodyType
{
	dynamicBody,
	staticBody
};

struct RigidbodyDef
{
	RigidbodyDef() : bodyType(RigidbodyType::dynamicBody), position(), eulerAngle(), linearVelocity(), angularVelocity(), gravityScale(1.0f), linearDamping(0.0f), angularDamping(0.0f) {}

	RigidbodyType bodyType;
	Vector3 position;
	Vector3 eulerAngle;
	Vector3 linearVelocity;
	Vector3 angularVelocity;
	float32 gravityScale;
	float32 linearDamping;
	float32 angularDamping;
};

class Rigidbody
{
public:
	Rigidbody(const RigidbodyDef& def);
	~Rigidbody();
	Fixture* setFixture(const FixtureDef& fixtureDef);
	void resetFixture();
	void setTransform(const Vector3& position, const Vector3& eulerAngle);
	Vector3 getPosition() const;
	Vector3 getEuler() const;
	void getLocalToWorld(Matrix4x4* matrix) const;
	void getWorldToLocal(Matrix4x4* matrix) const;
	void synchronizeTransform(); // body position 갱신
	void synchronizeFixture(); // aabb 갱신
	bool isAwake() const { return isAwakeFlag; }
	RigidbodyType getBodyType() const { return bodyType; }

	void setAwake(bool value)
	{
		if (value)
		{
			sleepTimer = 0.0f;
			isAwakeFlag = true;
		}
		else
		{
			linearVelocity.setZero();
			angularVelocity.setZero();
			force.setZero();
			torque.setZero();
			isAwakeFlag = false;
		}
	}

private:
	friend class World;
	friend class Island;
	friend class Renderer;
	friend class Fixture;
	friend class ContactManager;
	friend class SISolver;
	friend class NNCGSolver;
	friend class Debug;

	RigidbodyType bodyType;

	// constants
	Fixture* fixture;
	float32 invMass;
	Vector3 centerOfMass;
	Matrix3x3 inertiaTensor;
	Matrix3x3 invInertiaTensor;

	float32 gravityScale;
	float32 linearDamping;
	float32 angularDamping;

	// state variables
	Vector3 bodyPosition;
	Vector3 massPosition;
	Quaternion orientation;

	Vector3 linearVelocity;
	Vector3 angularVelocity;

	// derived quantities
	Matrix3x3 invWorldInertiaTensor;
	Matrix3x3 orientationMatrix;

	// computed quantities
	Vector3 force;
	Vector3 torque;

	ContactRef* contacts;

	World* world;
	Rigidbody* next;
	Rigidbody* prev;

	// solver
	int32 islandID;
	bool isAwakeFlag;
	float32 sleepTimer;

	void updateTransformDependants();
};