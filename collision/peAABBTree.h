#pragma once

#include"../common/peSettings.h"
#include"../common/peEventHandler.h"
#include"peCollision.h"

#define NULL_NODE (-1)
#define AABB_EXTENT (0.1f)

class AABBTreeNode
{
public:
	int32 child1;
	int32 child2;
	AABB aabb;
	void* userData;
	int32 height;

	union
	{
		int32 parent;	// tree 에서 부모 노드
		int32 next;		// free list 에서 다음 노드
	};

	bool isLeaf()
	{
		return child1 == NULL_NODE;
	}
};

class AABBTree
{
public:
	AABBTree();
	~AABBTree();

	int32 createLeaf(const AABB& aabb, void* userData);
	void deleteLeaf(int32 leafID);
	void updateLeaf(int32 leafID, const AABB& aabb);

	void* getUserData(int32 leafID) const
	{
		return nodes[leafID].userData;
	}

	const AABB& getAABB(int32 leafID) const
	{
		return nodes[leafID].aabb;
	}

	void query(IEventHandler<int32>* eventHandler, const AABB& aabb);

private:
	friend class PeTest;
	friend class Debug;

	int32 nodeCapacity;
	AABBTreeNode* nodes;
	int32 freeList;

	int32 root;
	int32 nodeCount;

	int32 allocNode();
	void freeNode(int32 nodeID);
	void insertLeaf(int32 leaf);
	void removeLeaf(int32 leaf);

	int32 balance(int32 iA);
};