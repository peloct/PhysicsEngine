#include"peJoint.h"
#include"../memory/peBlockAllocator.h"
#include"peDistanceJoint.h"
#include<new>

Joint* Joint::createJoint(BlockAllocator* blockAllocator, const JointDef& def)
{
	void* mem = nullptr;
	switch (def.jointType)
	{
	case JointType::eDistanceJoint:
		mem = blockAllocator->allocate(sizeof(DistanceJoint));
		return new (mem)DistanceJoint(static_cast<const DistanceJointDef&>(def));
	} 

	return nullptr;
}

void Joint::deleteJoint(BlockAllocator* blockAllocator, Joint* joint)
{
	joint->~Joint();
	switch (joint->jointType)
	{
	case JointType::eDistanceJoint:
		blockAllocator->free(joint, sizeof(DistanceJoint));
	default:
		break;
	}
}