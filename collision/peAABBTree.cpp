#include"peAABBTree.h"
#include<memory.h>
#include"../common/peGrowableStack.h"

// box2d Âü°í

AABBTree::AABBTree() : nodeCapacity(16), nodeCount(0)
{
	nodes = (AABBTreeNode*)peAlloc(nodeCapacity * sizeof(AABBTreeNode));
	freeList = 0;
	root = NULL_NODE;
	memset(nodes, 0, nodeCapacity * sizeof(AABBTreeNode));

	for (int i = 0; i < nodeCapacity - 1; ++i)
	{
		AABBTreeNode* each = nodes + i;
		each->next = i + 1;
	}

	AABBTreeNode* last = nodes + nodeCapacity - 1;
	last->next = NULL_NODE;
}

AABBTree::~AABBTree()
{
	peFree(nodes);
}

int32 AABBTree::allocNode()
{
	if (freeList == NULL_NODE)
	{
		assert(nodeCount == nodeCapacity);

		nodeCapacity *= 2;
		AABBTreeNode* newBuffer = (AABBTreeNode*)peAlloc(nodeCapacity * sizeof(AABBTreeNode));
		memcpy(newBuffer, nodes, nodeCount * sizeof(AABBTreeNode));

		peFree(nodes);
		nodes = newBuffer;

		for (int i = nodeCount; i < nodeCapacity - 1; ++i)
		{
			AABBTreeNode* each = nodes + i;
			each->next = i + 1;
			each->height = -1;
		}

		nodes[nodeCapacity - 1].next = NULL_NODE;
		nodes[nodeCapacity - 1].height = -1;
		freeList = nodeCount;
	}

	int32 ret = freeList;
	AABBTreeNode* newNode = nodes + freeList;
	freeList = newNode->next;

	newNode->height = 0;
	newNode->parent = NULL_NODE;
	newNode->child1 = NULL_NODE;
	newNode->child2 = NULL_NODE;
	newNode->userData = nullptr;

	++nodeCount;

	return ret;
}

void AABBTree::freeNode(int32 nodeID)
{
	nodes[nodeID].height = -1;
	nodes[nodeID].child1 = NULL_NODE;
	nodes[nodeID].child2 = NULL_NODE;
	nodes[nodeID].next = freeList;
	freeList = nodeID;

	--nodeCount;
}

int32 AABBTree::createLeaf(const AABB& aabb, void* userData)
{
	int32 leafNodeID = allocNode();
	AABBTreeNode* leafNode = nodes + leafNodeID;

	Vector3 extent(AABB_EXTENT, AABB_EXTENT, AABB_EXTENT);

	leafNode->aabb.minPos = aabb.minPos - extent;
	leafNode->aabb.maxPos = aabb.maxPos + extent;
	leafNode->userData = userData;

	insertLeaf(leafNodeID);

	return leafNodeID;
}

void AABBTree::deleteLeaf(int32 leafID)
{
	removeLeaf(leafID);
	freeNode(leafID);
}

void AABBTree::updateLeaf(int32 leafID, const AABB& aabb)
{
	assert(leafID >= 0 && leafID < nodeCapacity);

	Vector3 extent(AABB_EXTENT, AABB_EXTENT, AABB_EXTENT);

	removeLeaf(leafID);

	AABBTreeNode* leafNode = nodes + leafID;
	leafNode->aabb.minPos = aabb.minPos - extent;
	leafNode->aabb.maxPos = aabb.maxPos + extent;

	insertLeaf(leafID);
}

void AABBTree::insertLeaf(int32 leafID)
{
	if (root == NULL_NODE)
	{
		root = leafID;
		nodes[leafID].parent = NULL_NODE;
		return;
	}

	AABB leafAABB = nodes[leafID].aabb;
	int32 index = root;

	// box2d ÀÇ sibling Å½»ö ÈÞ¸®½ºÆ½ »ç¿ë, Á» ´õ ÁÁÀº ÈÞ¸®½ºÆ½ÀÌ ÀÖÀ»±î
	// http://allenchou.net/2014/02/game-physics-broadphase-dynamic-aabb-tree/ Âü°í
	while (!nodes[index].isLeaf())
	{
		int32 child1 = nodes[index].child1;
		int32 child2 = nodes[index].child2;

		AABB combined1;
		combined1.combine(nodes[child1].aabb, leafAABB);
		float32 volumeDiff1 = combined1.getVolume() - nodes[child1].aabb.getVolume();

		AABB combined2;
		combined2.combine(nodes[child2].aabb, leafAABB);
		float32 volumeDiff2 = combined2.getVolume() - nodes[child2].aabb.getVolume();

		if (volumeDiff1 < volumeDiff2)
			index = child1;
		else
			index = child2;
	}

	int32 sibling = index;
	int32 oldParent = nodes[sibling].parent;
	int32 newParent = allocNode();
	nodes[newParent].parent = oldParent;
	nodes[newParent].userData = nullptr;
	nodes[newParent].aabb.combine(leafAABB, nodes[sibling].aabb);
	nodes[newParent].height = nodes[sibling].height + 1;

	if (oldParent != NULL_NODE)
	{
		if (nodes[oldParent].child1 == sibling)
			nodes[oldParent].child1 = newParent;
		else
			nodes[oldParent].child2 = newParent;

		nodes[newParent].child1 = sibling;
		nodes[newParent].child2 = leafID;
		nodes[sibling].parent = newParent;
		nodes[leafID].parent = newParent;
	}
	else
	{
		nodes[newParent].child1 = sibling;
		nodes[newParent].child2 = leafID;
		nodes[sibling].parent = newParent;
		nodes[leafID].parent = newParent;
		root = newParent;
	}

	index = nodes[leafID].parent;
	while (index != NULL_NODE)
	{
		index = balance(index);

		int32 child1 = nodes[index].child1;
		int32 child2 = nodes[index].child2;

		assert(child1 != NULL_NODE);
		assert(child2 != NULL_NODE);

		nodes[index].height = 1 + peMaxf(nodes[child1].height, nodes[child2].height);
		nodes[index].aabb.combine(nodes[child1].aabb, nodes[child2].aabb);

		index = nodes[index].parent;
	}
}

void AABBTree::removeLeaf(int32 leafID)
{
	if (leafID == root)
	{
		root = NULL_NODE;
		return;
	}

	int32 parent = nodes[leafID].parent;
	int32 grandParent = nodes[parent].parent;
	int32 sibling;

	if (nodes[parent].child1 == leafID)
		sibling = nodes[parent].child2;
	else
		sibling = nodes[parent].child1;

	if (grandParent != NULL_NODE)
	{
		if (nodes[grandParent].child1 == parent)
			nodes[grandParent].child1 = sibling;
		else
			nodes[grandParent].child2 = sibling;

		nodes[sibling].parent = grandParent;
		freeNode(parent);

		int32 index = grandParent;
		while (index != NULL_NODE)
		{
			index = balance(index);

			int32 child1 = nodes[index].child1;
			int32 child2 = nodes[index].child2;

			nodes[index].aabb.combine(nodes[child1].aabb, nodes[child2].aabb);
			nodes[index].height = 1 + peMax(nodes[child1].height, nodes[child2].height);

			index = nodes[index].parent;
		}
	}
	else
	{
		root = sibling;
		nodes[sibling].parent = NULL_NODE;
		freeNode(parent);
	}
}

int32 AABBTree::balance(int32 iA)
{
	assert(iA != NULL_NODE);

	AABBTreeNode * A = nodes + iA;
	if (A->isLeaf() || A->height < 2)
	{
		return iA;
	}

	int32 iB = A->child1;
	int32 iC = A->child2;
	assert(0 <= iB && iB < nodeCapacity);
	assert(0 <= iC && iC < nodeCapacity);

	AABBTreeNode* B = nodes + iB;
	AABBTreeNode* C = nodes + iC;

	int32 balance = C->height - B->height;

	if (balance > 1)
	{
		int32 iF = C->child1;
		int32 iG = C->child2;
		AABBTreeNode* F = nodes + iF;
		AABBTreeNode* G = nodes + iG;
		assert(0 <= iF && iF < nodeCapacity);
		assert(0 <= iG && iG < nodeCapacity);

		C->child1 = iA;
		C->parent = A->parent;
		A->parent = iC;

		if (C->parent != NULL_NODE)
		{
			if (nodes[C->parent].child1 == iA)
			{
				nodes[C->parent].child1 = iC;
			}
			else
			{
				assert(nodes[C->parent].child2 == iA);
				nodes[C->parent].child2 = iC;
			}
		}
		else
		{
			root = iC;
		}

		if (F->height > G->height)
		{
			C->child2 = iF;
			A->child2 = iG;
			G->parent = iA;
			A->aabb.combine(B->aabb, G->aabb);
			C->aabb.combine(A->aabb, F->aabb);

			A->height = 1 + peMax(B->height, G->height);
			C->height = 1 + peMax(A->height, F->height);
		}
		else
		{
			C->child2 = iG;
			A->child2 = iF;
			F->parent = iA;
			A->aabb.combine(B->aabb, F->aabb);
			C->aabb.combine(A->aabb, G->aabb);

			A->height = 1 + peMax(B->height, F->height);
			C->height = 1 + peMax(A->height, G->height);
		}

		return iC;
	}

	if (balance < -1)
	{
		int32 iD = B->child1;
		int32 iE = B->child2;
		AABBTreeNode* D = nodes + iD;
		AABBTreeNode* E = nodes + iE;
		assert(0 <= iD && iD < nodeCapacity);
		assert(0 <= iE && iE < nodeCapacity);

		B->child1 = iA;
		B->parent = A->parent;
		A->parent = iB;

		if (B->parent != NULL_NODE)
		{
			if (nodes[B->parent].child1 == iA)
			{
				nodes[B->parent].child1 = iB;
			}
			else
			{
				assert(nodes[B->parent].child2 == iA);
				nodes[B->parent].child2 = iB;
			}
		}
		else
		{
			root = iB;
		}

		if (D->height > E->height)
		{
			B->child2 = iD;
			A->child1 = iE;
			E->parent = iA;
			A->aabb.combine(C->aabb, E->aabb);
			B->aabb.combine(A->aabb, D->aabb);

			A->height = 1 + peMax(C->height, E->height);
			B->height = 1 + peMax(A->height, D->height);
		}
		else
		{
			B->child2 = iE;
			A->child1 = iD;
			D->parent = iA;
			A->aabb.combine(C->aabb, D->aabb);
			B->aabb.combine(A->aabb, E->aabb);

			A->height = 1 + peMax(C->height, D->height);
			B->height = 1 + peMax(A->height, E->height);
		}

		return iB;
	}

	return iA;
}

void AABBTree::query(IEventHandler<int32>* eventHandler, const AABB& aabb)
{
	GrowableStack<int32, 256> stack;

	if (root == NULL_NODE)
		return;

	stack.push(root);
	while (!stack.empty())
	{
		int32 cur = stack.pop();
		AABBTreeNode* each = nodes + cur;

		if (!AABB::isOverlapped(each->aabb, aabb))
			continue;

		if (each->isLeaf())
		{
			eventHandler->Invoke(cur);
		}
		else
		{
			stack.push(each->child1);
			stack.push(each->child2);
		}
	}
}