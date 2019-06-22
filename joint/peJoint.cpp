#include"peJoint.h"
#include"../memory/peBlockAllocator.h"
#include"peSpringJoint.h"
#include<new>

Joint* Joint::createJoint(BlockAllocator* blockAllocator, const JointDef& def)
{
	void* mem = nullptr;
	switch (def.jointType)
	{
	case JointType::eSpringJoint:
		mem = blockAllocator->allocate(sizeof(SpringJoint));
		return new (mem)SpringJoint(static_cast<const SpringJointDef&>(def));
	} 

	return nullptr;
}

void Joint::deleteJoint(BlockAllocator* blockAllocator, Joint* joint)
{
	joint->~Joint();
	switch (joint->jointType)
	{
	case JointType::eSpringJoint:
		blockAllocator->free(joint, sizeof(SpringJoint));
	default:
		break;
	}
}