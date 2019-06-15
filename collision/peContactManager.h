#pragma once

#include"peBroadPhase.h"
#include"peContact.h"

class ContactManager
{
public:
	ContactManager(BlockAllocator* boxAllocator);
	static void addContact(ContactManager* contactManager, Fixture* fixtureA, Fixture* fixtureB);
	void removeContact(Contact* contact);
	void updateContact(); // broadphase �� �ǽ�
	void checkCollision(); // contact ���� collision ���

private:
	friend class World;
	friend class Rigidbody;
	friend class Debug;

	BroadPhase broadPhase;
	BlockAllocator* boxAllocator;
	int32 contactCount;
	Contact* contacts;
};