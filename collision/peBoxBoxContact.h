#pragma once

#include"peContact.h"

class BoxBoxContact : public Contact
{
public:
	void evaluate() override;

private:
	friend class Contact;

	BoxBoxContact(Fixture* fixtureA, Fixture* fixtureB) : Contact(fixtureA, fixtureB) {}

	static Contact* createFunc(BlockAllocator* blockAllocator, Fixture* fixtureA, Fixture* fixtureB);
	static void deleteFunc(BlockAllocator* blockAllocator, Contact* contact);
};