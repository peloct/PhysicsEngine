#include"peRigidbody.h"
#include"peWorld.h"
#include"peFixture.h"
#include<new>

Rigidbody::Rigidbody(const RigidbodyDef& def)
{
	bodyType = def.bodyType;

	fixture = nullptr;
	invMass = 0.0f;
	centerOfMass.setZero();
	inertiaTensor.setZero();
	invInertiaTensor.setZero();

	gravityScale = def.gravityScale;
	linearDamping = def.linearDamping;
	angularDamping = def.angularDamping;

	bodyPosition = def.position;
	massPosition = def.position;
	orientation = Quaternion::getEuler(def.eulerAngle);

	linearVelocity = def.linearVelocity;
	angularVelocity = def.angularVelocity;

	invWorldInertiaTensor.setZero();
	orientation.getOrientationMatrix(&orientationMatrix);

	force.setZero();
	torque.setZero();

	next = nullptr;
	prev = nullptr;

	world = nullptr;
	contacts = nullptr;

	islandID = -1;
	isAwakeFlag = true;
	sleepTimer = 0.0f;
	islandInfoID = -1;
}

Rigidbody::~Rigidbody()
{
	ContactRef* contactRef = contacts;
	while (contactRef)
	{
		world->contactManager.removeContact(contactRef->contact);
		contactRef = contactRef->next;
	}

	if (fixture != nullptr)
		resetFixture();
}

Fixture* Rigidbody::setFixture(const FixtureDef& fixtureDef)
{
	if (fixture != nullptr)
		resetFixture();

	void* mem = world->blockAllocator.allocate(sizeof(Fixture));
	fixture = new (mem) Fixture(fixtureDef, this, &world->contactManager.broadPhase);

	if (bodyType == RigidbodyType::staticBody)
	{
		invMass = 0.0f;
		centerOfMass.setZero();
		inertiaTensor.setZero();
		invInertiaTensor.setZero();
		invWorldInertiaTensor.setZero();
		return fixture;
	}

	MassData massData;
	fixture->computeMassData(&massData);

	invMass = 1 / massData.mass;
	centerOfMass = massData.centerOfMass;
	inertiaTensor = massData.inertiaTensor;
	invInertiaTensor = inertiaTensor.getInverse();
	invWorldInertiaTensor = orientationMatrix * invInertiaTensor * orientationMatrix.getTranspose();
	massPosition = bodyPosition + orientationMatrix * centerOfMass;

	return fixture;
}

void Rigidbody::resetFixture()
{
	if (fixture == nullptr)
		return;

	fixture->~Fixture();
	world->blockAllocator.free(fixture, sizeof(Fixture));
	fixture = nullptr;
}

void Rigidbody::setTransform(const Vector3& position, const Vector3& eulerAngle)
{
	bodyPosition = position;
	orientation = Quaternion::getEuler(eulerAngle);
	orientation.getOrientationMatrix(&orientationMatrix);
	massPosition = bodyPosition + orientationMatrix * centerOfMass;

	if (fixture == nullptr)
		return;

	fixture->synchronize();
}

Vector3 Rigidbody::getPosition() const
{
	return bodyPosition;
}

Vector3 Rigidbody::getEuler() const
{
	return orientation.toEulerAngle();
}

void Rigidbody::getLocalToWorld(Matrix4x4* matrix) const
{
	matrix->setTransform(orientationMatrix, bodyPosition);
}

void Rigidbody::getWorldToLocal(Matrix4x4* matrix) const
{
	matrix->setInverseTransform(orientationMatrix, bodyPosition);
}

void Rigidbody::synchronizeTransform()
{
	bodyPosition = massPosition + orientationMatrix.transform(-centerOfMass);
}

void Rigidbody::synchronizeFixture()
{
	if (fixture == nullptr)
		return;

	fixture->synchronize();
}

void Rigidbody::updateTransformDependants()
{
	orientation.getOrientationMatrix(&orientationMatrix);
	invWorldInertiaTensor = orientationMatrix * invInertiaTensor * orientationMatrix.getTranspose();
}