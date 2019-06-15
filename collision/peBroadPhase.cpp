#include"peBroadPhase.h"
#include"../peFixture.h"
#include<memory.h>
#include<algorithm>

using namespace std;

BroadPhase::BroadPhase()
{
	dirtyAABBCount = 0;
	dirtyAABBCapacity = 16;
	dirtyAABBs = (int32*)peAlloc(dirtyAABBCapacity * sizeof(int32));

	contactPairCount = 0;
	contactPairCapacity = 16 * 16;
	contactPairs = (ContactPair*)peAlloc(contactPairCapacity * sizeof(ContactPair));

	curQueryingAABBID = NULL_AABB_ID;
}

BroadPhase::~BroadPhase()
{
	peFree(dirtyAABBs);
}

int32 BroadPhase::insertAABB(const AABB& aabb, Fixture* fixture)
{
	int32 aabbID = aabbTree.createLeaf(aabb, (void*)fixture);
	setDirty(aabbID);
	return aabbID;
}

void BroadPhase::updateAABB(int32 aabbID, const AABB& aabb)
{
	const AABB& fatAABB = aabbTree.getAABB(aabbID);

	if (fatAABB.contains(aabb))
		return;

	aabbTree.updateLeaf(aabbID, aabb);
	setDirty(aabbID);
}

void BroadPhase::removeAABB(int32 aabbID)
{
	aabbTree.deleteLeaf(aabbID);
	for (int i = 0; i < dirtyAABBCount; ++i)
		if (dirtyAABBs[i] == aabbID)
			dirtyAABBs[i] = NULL_AABB_ID;
}

void BroadPhase::updateContacts(IEventHandler<Fixture*, Fixture*>* eventHandler)
{
	contactPairCount = 0;
	EventHandler<BroadPhase*, int32> queryEventHandler(this, aabbTreeQueryCallback);

	for (int i = 0; i < dirtyAABBCount; ++i)
	{
		int32 each = dirtyAABBs[i];
		if (each == NULL_AABB_ID)
			continue;

		curQueryingAABBID = each;
		const AABB& aabb = aabbTree.getAABB(each);
		aabbTree.query(&queryEventHandler, aabb);
	}

	dirtyAABBCount = 0;

	sort(contactPairs, contactPairs + contactPairCount);

	int i = 0;
	while (i < contactPairCount)
	{
		ContactPair* pair = contactPairs + i;
		void* userData1 = aabbTree.getUserData(pair->a);
		void* userData2 = aabbTree.getUserData(pair->b);

		eventHandler->Invoke((Fixture*)userData1, (Fixture*)userData2);

		++i;
		while (i < contactPairCount)
		{
			ContactPair* next = contactPairs + i;
			if (next->a == pair->a && next->b == pair->b)
				++i;
			else
				break;
		}
	}
}

void BroadPhase::setDirty(int32 aabbID)
{
	if (dirtyAABBCount == dirtyAABBCapacity)
	{
		dirtyAABBCapacity *= 2;
		int32* newBuffer = (int32*)peAlloc(dirtyAABBCapacity * sizeof(int32));
		memset(newBuffer, NULL_AABB_ID, dirtyAABBCapacity * sizeof(int32));
		memcpy(newBuffer, dirtyAABBs, dirtyAABBCount * sizeof(int32));
		peFree(dirtyAABBs);
		dirtyAABBs = newBuffer;
	}

	dirtyAABBs[dirtyAABBCount] = aabbID;
	++dirtyAABBCount;
}

void BroadPhase::aabbTreeQueryCallback(BroadPhase* bp, int32 aabbID)
{
	if (bp->curQueryingAABBID == aabbID)
		return;

	if (bp->contactPairCount == bp->contactPairCapacity)
	{
		bp->contactPairCapacity *= 2;
		ContactPair* newBuffer = (ContactPair*)peAlloc(bp->contactPairCapacity * sizeof(ContactPair));
		memcpy(newBuffer, bp->contactPairs, bp->contactPairCount * sizeof(ContactPair));
		peFree(bp->contactPairs);
		bp->contactPairs = newBuffer;
	}

	int32 a = bp->curQueryingAABBID;
	int32 b = aabbID;

	if (a > b)
		swap(a, b);

	bp->contactPairs[bp->contactPairCount].a = a;
	bp->contactPairs[bp->contactPairCount].b = b;
	bp->contactPairCount += 1;
}