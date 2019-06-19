#include"peWorld.h"
#include"collision/peContact.h"
#include"solver/peIsland.h"
#include"common/peTimer.h"
#include"common/peTimeStep.h"
#include<new>
#include<memory.h>

#define NULL_ISLAND -1

World::World() : contactManager(&blockAllocator), gravity(), profile()
{
	rigidbodyCount = 0;
	rigidbodies = nullptr;
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
	Timer timer;
	profile.reset();

	if (dt == 0.0f)
		return;

	TimeStep timeStep;
	timeStep.dt = dt;
	timeStep.ratio = dt * prevDTInv;
	timeStep.velocityIteration = velocityIteration;
	timeStep.positionIteration = positionIteration;

	timer2.reset();
	timer.reset();
	contactManager.checkCollision();
	profile.checkCollision += timer.getMilliseconds();

	Island island(rigidbodyCount, contactManager.contactCount, &stackAllocator);
	IslandInfo* islandInfoBuffer = (IslandInfo*)stackAllocator.allocate(rigidbodyCount * sizeof(IslandInfo));

	for (Rigidbody* eachBody = rigidbodies; eachBody; eachBody = eachBody->next)
		eachBody->islandID = NULL_ISLAND;
	for (Contact* eachContact = contactManager.contacts; eachContact; eachContact = eachContact->next)
		eachContact->islandID = NULL_ISLAND;

	int32 stackSize = 0;
	Rigidbody** stack = (Rigidbody**)stackAllocator.allocate(rigidbodyCount * sizeof(Rigidbody*));

	int32 islandCount = 0;
	for (Rigidbody* eachBody = rigidbodies; eachBody; eachBody = eachBody->next)
	{
		if (eachBody->islandID != NULL_ISLAND)
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

				if (eachContact->islandID != NULL_ISLAND)
					continue;

				eachContact->islandID = island.addContact(eachContact);
				Rigidbody* otherBody = eachContactRef->other;

				if (otherBody->islandID != NULL_ISLAND)
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
				island.rigidbodies[i]->islandID = NULL_ISLAND;
		}
	}

	clearIslandInfo();
	for (int i = 0; i < islandCount; ++i)
		saveIslandInfo(islandInfoBuffer[i]);

	stackAllocator.free(stack);
	stackAllocator.free(islandInfoBuffer);

	for (Rigidbody* eachBody = rigidbodies; eachBody; eachBody = eachBody->next)
	{
		if (eachBody->islandID == NULL_ISLAND)
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