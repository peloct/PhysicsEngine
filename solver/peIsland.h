#pragma once

#include"../common/peMath.h"
#include"../common/peTimeStep.h"
#include"peNNCGSolver.h"

class Rigidbody;
class Contact;
class Joint;
class StackAllocator;
class Profile;

struct IslandInfo
{
	IslandInfo() : isValid(false), rigidbodyCount(0), nncgSolverPrevStepInfo() {}

	bool isValid;
	int32 rigidbodyCount;
	NNCGSolverStepInfo nncgSolverPrevStepInfo;
};

class Island
{
public:

	Island(int32 bodyCapacity, int32 contactCapacity, int32 jointCapacity, StackAllocator* stackAllocator);
	~Island();
	IslandInfo solve(const TimeStep& timeStep, const Vector3& gravity, const IslandInfo& islandInfo, Profile* profile);

	void clear()
	{
		rigidbodyCount = 0;
		contactCount = 0;
		jointCount = 0;
	}

	int32 addRigidbody(Rigidbody* rigidbody)
	{
		int32 id = rigidbodyCount++;
		rigidbodies[id] = rigidbody;
		return id;
	}

	int32 addContact(Contact* contact)
	{
		int32 id = contactCount++;
		contacts[id] = contact;
		return id;
	}

	int32 addJoint(Joint* joint)
	{
		int32 id = jointCount++;
		joints[id] = joint;
		return id;
	}

private:

	friend class World;

	int32 rigidbodyCount;
	Rigidbody** rigidbodies;

	int32 contactCount;
	Contact** contacts;

	int32 jointCount;
	Joint** joints;

	Vector3* positions;
	Quaternion* orientations;
	Vector3* linearVelocities;
	Vector3* angularVelocities;

	StackAllocator* stackAllocator;
};