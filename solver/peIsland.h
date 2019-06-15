#pragma once

#include"../common/peMath.h"
#include"../common/peTimeStep.h"

class Rigidbody;
class Contact;
class StackAllocator;
class Profile;

class Island
{
public:

	Island(int32 bodyCapacity, int32 contactCapacity, StackAllocator* stackAllocator);
	~Island();
	void solve(const TimeStep& timeStep, const Vector3& gravity, Profile* profile);

	void clear()
	{
		rigidbodyCount = 0;
		contactCount = 0;
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

private:

	friend class World;

	int32 rigidbodyCount;
	Rigidbody** rigidbodies;

	int32 contactCount;
	Contact** contacts;

	Vector3* positions;
	Quaternion* orientations;
	Vector3* linearVelocities;
	Vector3* angularVelocities;

	StackAllocator* stackAllocator;
};