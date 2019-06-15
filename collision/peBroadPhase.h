#pragma once

#include"../common/peMath.h"
#include"../common/peEventHandler.h"
#include"peCollision.h"
#include"peAABBTree.h"

class Fixture;

#define NULL_AABB_ID -1

class ContactPair
{
public:
	int32 a;
	int32 b;

	bool operator< (const ContactPair& rhs) const
	{
		if (a != rhs.a)
			return a < rhs.a;
		return b < rhs.b;
	}
};

class BroadPhase
{
public:
	BroadPhase();
	~BroadPhase();

	int32 insertAABB(const AABB& aabb, Fixture* fixture);
	void updateAABB(int32 aabbID, const AABB& aabb);
	void removeAABB(int32 aabbID);
	bool testOverlap(int32 aabbID1, int32 aabbID2) const
	{
		const AABB& aabb1 = aabbTree.getAABB(aabbID1);
		const AABB& aabb2 = aabbTree.getAABB(aabbID2);
		return AABB::isOverlapped(aabb1, aabb2);
	}

	void updateContacts(IEventHandler<Fixture*, Fixture*>* eventHandler);
	static void aabbTreeQueryCallback(BroadPhase* broadPhase, int32 aabbID);

private:
	friend class Debug;

	AABBTree aabbTree;

	int32 dirtyAABBCapacity;
	int32 dirtyAABBCount;

	int32* dirtyAABBs;
	int32 curQueryingAABBID;

	int32 contactPairCapacity;
	int32 contactPairCount;
	ContactPair* contactPairs;

	void setDirty(int32 aabbID);
};