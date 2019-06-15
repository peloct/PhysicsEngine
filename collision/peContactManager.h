#pragma once

#include"peBroadPhase.h"
#include"peContact.h"

class ContactManager
{
public:
	ContactManager(BlockAllocator* boxAllocator);
	static void addContact(ContactManager* contactManager, Fixture* fixtureA, Fixture* fixtureB);
	void removeContact(Contact* contact);
	void updateContact(); // broadphase 를 실시
	void checkCollision(); // contact 에서 collision 계산

private:
	friend class World;
	friend class Rigidbody;
	friend class Debug;

	BroadPhase broadPhase;
	BlockAllocator* boxAllocator;
	int32 contactCount;
	Contact* contacts;
};