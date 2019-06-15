#pragma once

#include"../common/peSettings.h"
#include"../shape/peShape.h"

class Contact;
class BlockAllocator;
class Rigidbody;
class Fixture;

typedef Contact* ContactCreateFunc(BlockAllocator* boxAllocator, Fixture* fixtureA, Fixture* fixtureB);
typedef void ContactDestroyFunc(BlockAllocator* boxAllocator, Contact* contact);

enum ContactFaceOwner
{
	fixtureA,
	fixtureB
};

struct ContactPoint
{
	Vector3 localPoint;
	float32 normalImpulse;
	Vector2 tangentImpulse;
};

struct ContactCacheKey
{
	ContactFaceOwner faceOwner;
	int32 contactCount;
	int32 a;
	int32 b;
	
	bool operator==(const ContactCacheKey& rhs) const
	{
		return faceOwner == rhs.faceOwner && contactCount == rhs.contactCount && a == rhs.a && b == rhs.b;
	}
};

class ContactRef
{
public:
	ContactRef() : contact(nullptr), other(nullptr), prev(nullptr), next(nullptr) {}

	Contact* contact;
	Rigidbody* other;
	ContactRef* prev;
	ContactRef* next;
};

struct ContactFactory
{
	ContactCreateFunc* createFunc;
	ContactDestroyFunc* destroyFunc;
};

class Contact
{
protected:
	friend class World;
	friend class ContactManager;
	friend class ContactSolver;
	friend class Debug;

	Contact(Fixture* fixtureA, Fixture* fixtureB) : contactCacheKey()
	{
		this->fixtureA = fixtureA;
		this->fixtureB = fixtureB;
		prev = nullptr;
		next = nullptr;
		contactPointCount = 0;
		contactFaceOwner = ContactFaceOwner::fixtureA;
		friction = calcFriction();
		restitution = calcRestitution();
		islandID = -1;
	}

	Fixture* fixtureA;
	Fixture* fixtureB;

	ContactRef refFromA;
	ContactRef refFromB;

	ContactFaceOwner contactFaceOwner;
	Vector3 localPlanePoint;
	Vector3 localNormal;
	Vector3 localTangent1;
	Vector3 localTangent2;

	int32 contactPointCount;
	ContactCacheKey contactCacheKey;
	ContactPoint contactPoints[8];

	Contact* prev;
	Contact* next;

	float32 friction;
	float32 restitution;

	int32 islandID;

	float32 calcFriction() const;
	float32 calcRestitution() const;
	void updateContact();

	virtual void evaluate() = 0;

	static ContactFactory factoryMap[Shape::Type::shapeCount][Shape::Type::shapeCount];
	static bool isFactoryInitialized;

	static Contact* createContact(BlockAllocator* boxAllocator, Fixture* fixtureA, Fixture* fixtureB);
	static void deleteContact(BlockAllocator* boxAllocator, Contact* contact);
};