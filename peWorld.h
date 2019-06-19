#pragma once

#include"common/peSettings.h"
#include"common/peMath.h"
#include"peRigidbody.h"
#include"collision/peContactManager.h"
#include"memory/peStackAllocator.h"
#include"memory/peBlockAllocator.h"
#include"common/peProfile.h"

struct IslandInfo;

class World
{
public:
	World();
	~World();

	void step(float32 dt, int32 velocityIteration, int32 positionIteration);
	Rigidbody* createBody(const RigidbodyDef& def);
	void deleteBody(Rigidbody* body);
	void setGravity(const Vector3& gravity)
	{
		this->gravity = gravity;
	}

	const Profile& getProfile() const { return profile; }

private:
	friend class Rigidbody;
	friend class Debug;

	int32 rigidbodyCount;
	Rigidbody* rigidbodies;

	Vector3 gravity;
	float32 prevDTInv;

	ContactManager contactManager;
	StackAllocator stackAllocator;
	BlockAllocator blockAllocator;

	Profile profile;

	void clearIslandInfo() { islandInfoCount = 0; }
	void saveIslandInfo(const IslandInfo& info);
	int32 islandInfoCount;
	int32 islandInfoCapacity;
	IslandInfo* islandInfos;
};