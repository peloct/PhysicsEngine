#pragma once

#include"../collision/peContact.h"
#include"../common/peTimeStep.h"

class StackAllocator;

#define BOUNCE_THRESHOLD -1.0f
#define BAUMGARTE 0.2f
#define SOME_THRESHOLD -0.00001f

struct ContactSolverDef
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

struct ContactVelocityConstraintPoint
{
	Vector3 rA;
	Vector3 rB;
	float32 normalImpulse;
	Vector2 tangentImpulse;
	float32 velocityBias;
	float32 normalMass;
	Matrix2x2 tangentMass;
};

struct ContactVelocityConstraint
{
	int32 contactIndex;
	int32 indexA;
	int32 indexB;
	float32 invMassA;
	float32 invMassB;
	Matrix3x3 invWorldIA;
	Matrix3x3 invWorldIB;
	float32 friction;
	float32 restitution;
	Vector3 normal;
	Vector3 tangent1;
	Vector3 tangent2;
	int32 contactPointCount;
	ContactVelocityConstraintPoint contactPoints[8];
};

struct ContactPositionConstraint
{
	ContactFaceOwner faceOwner;
	int32 indexA;
	int32 indexB;
	float32 invMassA;
	float32 invMassB;
	Matrix3x3 invIA;
	Matrix3x3 invIB;
	Vector3 localCenterA;
	Vector3 localCenterB;
	Vector3 localNormal;
	Vector3 localPlanePoint;
	int32 contactPointCount;
	Vector3 contactPoints[8];
};

class ContactSolver
{
public:
	ContactSolver(const ContactSolverDef& def);
	~ContactSolver();

	void initVelocityConstraints();
	void warmStart();
	void solveVelocityConstraints();
	void saveImpulse();
	bool solvePositionConstraints();

private:
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