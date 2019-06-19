#pragma once

#include"peContactSolver.h"

class StackAllocator;

struct NNCGSolverDef
{
	int32 rigidbodyCount;
	TimeStep timeStep;
	int32 contactCount;
	Contact** contactsInIsland;
	StackAllocator* stackAllocator;
	Vector3* positions;
	Quaternion* orientations;
	Vector3* linearVelocities;
	Vector3* angularVelocities;
	float32 prevGradientMagSqr;
};

class NNCGSolver : public ContactSolver
{
public:
	NNCGSolver(const NNCGSolverDef& def);
	~NNCGSolver();

	bool hit;
	void initVelocityConstraints() override;
	void warmStart() override;
	void solveVelocityConstraints() override;
	void saveImpulse() override;
	void applyeDelta() override;
	bool solvePositionConstraints() override;
	float32 getCurGradientMagSqr() const { return curGradientMagSqr; }

private:
	struct ContactVelocityConstraintPoint;
	struct ContactVelocityConstraint;
	struct ContactPositionConstraint;

	StackAllocator* stackAllocator;

	int32 contactCount;
	Contact** contacts;
	Vector3* positions;
	Quaternion* orientations;
	Vector3* linearVelocities;
	Vector3* angularVelocities;

	int32 velConstraintsCount;
	ContactVelocityConstraint* velocityConstraints;

	int32 rigidbodyCount;
	Vector3* dLinearV;
	Vector3* dAngularV;

	float32 curGradientMagSqr;
	float32 prevGradientMagSqr;

	int32 posConstraintsCount;
	ContactPositionConstraint* positionConstraints;

	void PGS();
};