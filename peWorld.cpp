#include"peWorld.h"
#include"collision/peContact.h"
#include"solver/peIsland.h"
#include"common/peTimer.h"
#include"common/peTimeStep.h"
#include<new>
#include<memory.h>

World::World() : contactManager(&blockAllocator), gravity(), profile()
{
	rigidbodyCount = 0;
	rigidbodies = nullptr;
	jointCount = 0;
	joints = nullptr;

	prevDTInv = 0.0f;

	islandInfoCount = 0;
	islandInfoCapacity = 50;
	islandInfos = (IslandInfo*)peAlloc(islandInfoCapacity * sizeof(IslandInfo));
}

World::~World()
{
	while (rigidbodies)
		deleteBody(rigidbodies);
}

void World::step(float32 dt, int32 velocityIteration, int32 positionIteration)
{
	Timer timer2;
	timer2.reset();

	Timer timer;

	profile.reset();

	if (dt == 0.0f)
		return;

	TimeStep timeStep;
	timeStep.dt = dt;
	timeStep.ratio = dt * prevDTInv;
	timeStep.velocityIteration = velocityIteration;
	timeStep.positionIteration = positionIteration;

	timer.reset();
	contactManager.checkCollision();
	profile.checkCollision += timer.getMilliseconds();

	Island island(rigidbodyCount, contactManager.contactCount, jointCount, &stackAllocator);
	IslandInfo* islandInfoBuffer = (IslandInfo*)stackAllocator.allocate(rigidbodyCount * sizeof(IslandInfo));

	for (Rigidbody* eachBody = rigidbodies; eachBody; eachBody = eachBody->next)
		eachBody->islandID = NULL_ID;
	for (Contact* eachContact = contactManager.contacts; eachContact; eachContact = eachContact->next)
		eachContact->islandID = NULL_ID;
	for (Joint* eachJoint = joints; eachJoint; eachJoint = eachJoint = eachJoint->next)
		eachJoint->islandID = NULL_ID;

	int32 stackSize = 0;
	Rigidbody** stack = (Rigidbody**)stackAllocator.allocate(rigidbodyCount * sizeof(Rigidbody*));

	int32 islandCount = 0;

	for (Rigidbody* eachBody = rigidbodies; eachBody; eachBody = eachBody->next)
	{
		if (eachBody->islandID != NULL_ID)
			continue;

		if (eachBody->bodyType == RigidbodyType::staticBody)
			continue;

		if (eachBody->isAwake() == false)
			continue;

		island.clear();
		eachBody->islandID = island.addRigidbody(eachBody);
		stack[stackSize++] = eachBody;

		int32 islandInfoID = eachBody->islandInfoID;
		bool isIslandInfoValid = true;

		while (stackSize > 0)
		{
			Rigidbody* here = stack[--stackSize];

			if (islandInfoID != here->islandInfoID)
				isIslandInfoValid = false;

			if (here->bodyType == RigidbodyType::staticBody)
				continue;

			for (ContactRef* eachContactRef = here->contacts; eachContactRef; eachContactRef = eachContactRef->next)
			{
				Contact* eachContact = eachContactRef->contact;

				if (eachContact->isChanged)
					isIslandInfoValid = false;

				if (eachContact->contactPointCount == 0)
					continue;

				if (eachContact->islandID != NULL_ID)
					continue;

				eachContact->islandID = island.addContact(eachContact);
				Rigidbody* otherBody = eachContactRef->other;

				if (otherBody->islandID != NULL_ID)
					continue;

				otherBody->islandID = island.addRigidbody(otherBody);
				stack[stackSize++] = otherBody;
			}

			for (JointRef* eachJointRef = here->joints; eachJointRef; eachJointRef = eachJointRef->next)
			{
				Joint* eachJoint = eachJointRef->joint;
				
				if (eachJoint->islandID != NULL_ID)
					continue;

				eachJoint->islandID = island.addJoint(eachJoint);
				Rigidbody* otherBody = eachJointRef->other;

				if (otherBody == nullptr || otherBody->islandID != NULL_ID)
					continue;

				otherBody->islandID = island.addRigidbody(otherBody);
				stack[stackSize++] = otherBody;
			}
		}

		IslandInfo islandInfo;
		islandInfo.isValid = false;

		if (isIslandInfoValid && island.rigidbodyCount == islandInfos[islandInfoID].rigidbodyCount)
		{
			islandInfo = islandInfos[islandInfoID];
			islandInfo.isValid = true;
		}

		int32 newIslandInfoID = islandCount;
		profile.solveCount += velocityIteration;
		islandInfoBuffer[islandCount++] = island.solve(timeStep, gravity, islandInfo, &profile);

		for (int i = 0; i < island.rigidbodyCount; ++i)
		{
			island.rigidbodies[i]->islandInfoID = newIslandInfoID;
			if (island.rigidbodies[i]->bodyType == RigidbodyType::staticBody)
				island.rigidbodies[i]->islandID = NULL_ID;
		}
	}

	clearIslandInfo();
	for (int i = 0; i < islandCount; ++i)
		saveIslandInfo(islandInfoBuffer[i]);

	stackAllocator.free(stack);
	stackAllocator.free(islandInfoBuffer);

	for (Rigidbody* eachBody = rigidbodies; eachBody; eachBody = eachBody->next)
	{
		if (eachBody->islandID == NULL_ID)
			continue;

		timer.reset();
		eachBody->synchronizeFixture();
		profile.syncFixture += timer.getMilliseconds();

		eachBody->force.setZero();
		eachBody->torque.setZero();
	}

	prevDTInv = 1 / dt;

	timer.reset();
	contactManager.updateContact();
	profile.updateContacts += timer.getMilliseconds();
	profile.step += timer2.getMilliseconds();

	blockAllocator.getProfile(&profile);
}

Rigidbody* World::createBody(const RigidbodyDef& def)
{
	void* mem = blockAllocator.allocate(sizeof(Rigidbody));
	Rigidbody* body = new (mem) Rigidbody(def);
	body->world = this;
	
	if (rigidbodies != nullptr)
		rigidbodies->prev = body;
	body->next = rigidbodies;
	body->prev = nullptr;
	rigidbodies = body;

	++rigidbodyCount;

	return body;
}

void World::deleteBody(Rigidbody* body)
{
	body->~Rigidbody();

	if (rigidbodies == body)
		rigidbodies = body->next;
	if (body->next != nullptr)
		body->next->prev = body->prev;
	if (body->prev != nullptr)
		body->prev->next = body->next;

	--rigidbodyCount;

	blockAllocator.free(body, sizeof(Rigidbody));
}

Joint* World::createJoint(const JointDef& def)
{
	Joint* joint = Joint::createJoint(&blockAllocator, def);

	joint->next = joints;
	joint->prev = nullptr;
	if (joints)
		joints->prev = joint;
	joints = joint;

	Rigidbody* bodyA = joint->bodyA;
	Rigidbody* bodyB = joint->bodyB;

	joint->refFromA.joint = joint;
	joint->refFromA.other = nullptr;
	joint->refFromA.next = bodyA->joints;
	joint->refFromA.prev = nullptr;
	if (bodyA->joints)
		bodyA->joints->prev = &joint->refFromA;
	bodyA->joints = &joint->refFromA;

	if (bodyB)
	{
		joint->refFromA.other = bodyB;
		joint->refFromB.joint = joint;
		joint->refFromB.other = bodyA;
		joint->refFromB.next = bodyB->joints;
		joint->refFromB.prev = nullptr;
		if (bodyB->joints)
			bodyB->joints->prev = &joint->refFromB;
		bodyB->joints = &joint->refFromB;
	}

	++jointCount;
	return joint;
}

void World::deleteJoint(Joint* joint)
{
	if (joint->prev == nullptr)
	{
		joints = joint->next;
		if (joint->next)
			joint->next->prev = nullptr;
	}
	else
	{
		joint->prev->next = joint->next;
		if (joint->next)
			joint->next->prev = joint->prev;
	}

	if (joint->refFromA.prev == nullptr)
	{
		joint->bodyA->joints = joint->refFromA.next;
		if (joint->refFromA.next)
			joint->refFromA.next->prev = nullptr;
	}
	else
	{
		joint->refFromA.prev->next = joint->refFromA.next;
		if (joint->refFromA.next)
			joint->refFromA.next->prev = joint->refFromA.prev;
	}

	if (joint->refFromB.prev == nullptr)
	{
		joint->bodyB->joints = joint->refFromB.next;
		if (joint->refFromB.next)
			joint->refFromB.next->prev = nullptr;
	}
	else
	{
		joint->refFromB.prev->next = joint->refFromB.next;
		if (joint->refFromB.next)
			joint->refFromB.next->prev = joint->refFromB.prev;
	}

	Joint::deleteJoint(&blockAllocator, joint);
	--jointCount;
}

void World::saveIslandInfo(const IslandInfo& info)
{
	if (islandInfoCount == islandInfoCapacity)
	{
		islandInfoCapacity *= 2;
		IslandInfo* newBuffer = (IslandInfo*)peAlloc(islandInfoCapacity * sizeof(IslandInfo));
		memcpy(newBuffer, islandInfos, islandInfoCount * sizeof(IslandInfo));
		peFree(islandInfos);
		islandInfos = newBuffer;
	}

	islandInfos[islandInfoCount++] = info;
}