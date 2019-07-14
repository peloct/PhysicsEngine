#include"peContact.h"
#include"../peFixture.h"
#include"../memory/peBlockAllocator.h"
#include"../peRigidbody.h"
#include"peBoxBoxContact.h"

ContactFactory Contact::factoryMap[Shape::Type::shapeCount][Shape::Type::shapeCount];
bool Contact::isFactoryInitialized = false;
int32 Contact::nextGUID = 0;

float32 Contact::calcFriction() const
{
	return  peMaxf(fixtureA->getFriction(), fixtureB->getFriction());
}

float32 Contact::calcRestitution() const
{
	return peMaxf(fixtureA->getRestitution(), fixtureB->getRestitution());
}

void Contact::updateContact()
{
	isChanged = false;

	Rigidbody* bodyA = fixtureA->getRigidbody();
	Rigidbody* bodyB = fixtureB->getRigidbody();

	ContactCacheKey prevKey = contactCacheKey;
	ContactPoint points[8];
	
	for (int i = 0; i < 8; ++i)
	{
		if (i < contactPointCount)
		{
			points[i] = contactPoints[i];
		}
		else
		{
			points[i].normalImpulse = 0;
			points[i].tangentImpulse.setZero();
			points[i].direction.setZero();
			points[i].fGradient.setZero();
		}
	}

	bool wasTouching = contactPointCount > 0;
	bool touchingStateChanged = false;

	evaluate();

	if (prevKey == contactCacheKey)
	{
		for (int i = 0; i < contactPointCount; ++i)
		{
			contactPoints[i].normalImpulse = points[i].normalImpulse;
			contactPoints[i].tangentImpulse = points[i].tangentImpulse;
			contactPoints[i].direction = points[i].direction;
			contactPoints[i].fGradient = points[i].fGradient;
		}
	}
	else
	{
		touchingStateChanged = true;
	}

	bool isTouching = contactPointCount > 0;

	if (wasTouching != isTouching)
	{
		touchingStateChanged = true;
		bodyA->setAwake(true);
		bodyB->setAwake(true);
	}

	if (touchingStateChanged)
		isChanged = true;
}

Contact* Contact::createContact(BlockAllocator* blockAllocator, Fixture* fixtureA, Fixture* fixtureB)
{
	if (!isFactoryInitialized)
	{
		isFactoryInitialized = true;
		factoryMap[Shape::Type::box][Shape::Type::box].createFunc = BoxBoxContact::createFunc;
		factoryMap[Shape::Type::box][Shape::Type::box].destroyFunc = BoxBoxContact::deleteFunc;
	}

	Shape::Type shapeTypeA = fixtureA->getShape()->getType();
	Shape::Type shapeTypeB = fixtureB->getShape()->getType();

	return factoryMap[shapeTypeA][shapeTypeB].createFunc(blockAllocator, fixtureA, fixtureB);
}

void Contact::deleteContact(BlockAllocator* blockAllocator, Contact* contact)
{
	assert(isFactoryInitialized);

	Shape::Type shapeTypeA = contact->fixtureA->getShape()->getType();
	Shape::Type shapeTypeB = contact->fixtureB->getShape()->getType();

	factoryMap[shapeTypeA][shapeTypeB].destroyFunc(blockAllocator, contact);
}

static bool isFactoryInitialized = false;