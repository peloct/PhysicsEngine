#include"peContactManager.h"
#include"../peRigidbody.h"
#include"../peFixture.h"
#include"peContact.h"

ContactManager::ContactManager(BlockAllocator* boxAllocator)
{
	contacts = nullptr;
	contactCount = 0;
	this->boxAllocator = boxAllocator;
}

void ContactManager::addContact(ContactManager* cm, Fixture* fixtureA, Fixture* fixtureB)
{
	// 이미 두 fixture 사이에 contact 가 있으면 무시할 필요가 있다.

	ContactRef* contactRef = fixtureB->rigidbody->contacts;

	while (contactRef)
	{
		if (contactRef->contact->fixtureA == fixtureA && contactRef->contact->fixtureB == fixtureB)
			return;
		if (contactRef->contact->fixtureA == fixtureB && contactRef->contact->fixtureB == fixtureA)
			return;

		contactRef = contactRef->next;
	}

	Contact* contact = Contact::createContact(cm->boxAllocator, fixtureA, fixtureB);
	// contact 를 생성하는 과정에서 swaping 이 있을 수 있다.
	fixtureA = contact->fixtureA;
	fixtureB = contact->fixtureB;

	contact->next = cm->contacts;
	contact->prev = nullptr;
	if (cm->contacts)
		cm->contacts->prev = contact;
	cm->contacts = contact;

	Rigidbody* bodyA = fixtureA->rigidbody;
	Rigidbody* bodyB = fixtureB->rigidbody;

	contact->refFromA.contact = contact;
	contact->refFromA.next = bodyA->contacts;
	contact->refFromA.prev = nullptr;
	if (bodyA->contacts)
		bodyA->contacts->prev = &contact->refFromA;
	bodyA->contacts = &contact->refFromA;
	bodyA->contacts->other = bodyB;

	contact->refFromB.contact = contact;
	contact->refFromB.next = bodyB->contacts;
	contact->refFromB.prev = nullptr;
	if (bodyB->contacts)
		bodyB->contacts->prev = &contact->refFromB;
	bodyB->contacts = &contact->refFromB;
	bodyB->contacts->other = bodyA;

	++cm->contactCount;
}

void ContactManager::removeContact(Contact* contact)
{
	if (contact->prev == nullptr)
	{
		contacts = contact->next;
		if (contact->next)
			contact->next->prev = nullptr;
	}
	else
	{
		contact->prev->next = contact->next;
		if (contact->next)
			contact->next->prev = contact->prev;
	}

	if (contact->refFromA.prev == nullptr)
	{
		contact->fixtureA->rigidbody->contacts = contact->refFromA.next;
		if (contact->refFromA.next)
			contact->refFromA.next->prev = nullptr;
	}
	else
	{
		contact->refFromA.prev->next = contact->refFromA.next;
		if (contact->refFromA.next)
			contact->refFromA.next->prev = contact->refFromA.prev;
	}
	
	if (contact->refFromB.prev == nullptr)
	{
		contact->fixtureB->rigidbody->contacts = contact->refFromB.next;
		if (contact->refFromB.next)
			contact->refFromB.next->prev = nullptr;
	}
	else
	{
		contact->refFromB.prev->next = contact->refFromB.next;
		if (contact->refFromB.next)
			contact->refFromB.next->prev = contact->refFromB.prev;
	}

	Contact::deleteContact(boxAllocator, contact);
	--contactCount;
}

void ContactManager::updateContact()
{
	EventHandler<ContactManager*, Fixture*, Fixture*> eventHandler(this, addContact);
	broadPhase.updateContacts(&eventHandler);
}

void ContactManager::checkCollision()
{
	Contact* each = contacts;

	while (each)
	{
		if (!broadPhase.testOverlap(each->fixtureA->aabbID, each->fixtureB->aabbID))
		{
			Contact* contactToRemove = each;
			each = each->next;
			removeContact(contactToRemove);
			continue;
		}

		each->updateContact();
		each = each->next;
	}
}