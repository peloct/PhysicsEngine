#pragma once

#include"..//common/peSettings.h"
#include"..//common/peMath.h"

class BlockAllocator;
class Fixture;
class Joint;
class Rigidbody;
class VelocityConstraint;
class NNCGSolverData;
class ContactPositionConstraint;

enum JointType
{
	eSpringJoint,
	eTypeCount
};

struct JointDef
{
	JointDef(JointType jointType) : jointType(jointType) {}

	JointType jointType;
};

struct JointRef
{
	JointRef() : joint(nullptr), other(nullptr), prev(nullptr), next(nullptr) {}

	Joint* joint;
	Rigidbody* other;
	JointRef* prev;
	JointRef* next;
};

class Joint
{
public:
	JointType getJointType() const { return jointType; }

protected:
	friend class NNCGSolver;
	friend class World;

	Joint(JointType jointType, Rigidbody* bodyA, Rigidbody* bodyB)
		: jointType(jointType), bodyA(bodyA), bodyB(bodyB), refFromA(), refFromB(), prev(nullptr), next(nullptr), islandID(NULL_ID) {}

	virtual ~Joint() {}

	JointType jointType;

	Rigidbody* bodyA;
	Rigidbody* bodyB;

	JointRef refFromA;
	JointRef refFromB;

	Joint* prev;
	Joint* next;

	int32 islandID;

	virtual int32 getVelConstraintCount() const = 0;
	virtual void createVelConstraints(const NNCGSolverData& solverData, VelocityConstraint* velocityConstraints, Vector3* dLinearV, Vector3* dAngularV) = 0;
	virtual void saveImpulse(VelocityConstraint* velocityConstraints) = 0;
	virtual bool solvePosConstraints(Vector3* positions, Quaternion* orientations) const = 0;

	static Joint* createJoint(BlockAllocator* blockAllocator, const JointDef& def);
	static void deleteJoint(BlockAllocator* blockAllocator, Joint* joint);
};