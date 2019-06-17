#pragma once

#include"peContactSolver.h"

class StackAllocator;

struct SISolverDef
{
	TimeStep timeStep;
	int32 contactCount;
	Contact** contactsInIsland;
	StackAllocator* stackAllocator;
	Vector3* positions;
	Quaternion* orientations;
	Vector3* linearVelocities;
	Vector3* angularVelocities;
};

class SISolver : public ContactSolver
{
public:
	SISolver(const SISolverDef& def);
	~SISolver();

	void initVelocityConstraints() override;
	void warmStart() override;
	void solveVelocityConstraints() override;
	void saveImpulse() override;
	bool solvePositionConstraints() override;

private:
	struct ContactVelocityConstraintPoint;
	struct ContactVelocityConstraint;
	struct ContactPositionConstraint;

	StackAllocator* stackAllocator;
	int32 constraintsCount;
	Contact** contacts;
	ContactVelocityConstraint* velocityConstraints;
	ContactPositionConstraint* positionConstraints;
	Vector3* positions;
	Quaternion* orientations;
	Vector3* linearVelocities;
	Vector3* angularVelocities;
};