#pragma once

#include"..//common/peTimeStep.h"
#include"..//collision/peContact.h"

class StackAllocator;
class Joint;

struct Jsp
{
	Vector3 linearA;
	Vector3 angularA;
	Vector3 linearB;
	Vector3 angularB;
};

struct Bsp
{
	Vector3 linearA;
	Vector3 angularA;
	Vector3 linearB;
	Vector3 angularB;
};

struct VelocityConstraint
{
	void* cacheRef;
	bool doSolve;

	int32 indexA;
	int32 indexB;
	float32 desiredDelV;
	float32 impulse;
	float32 minImpulse;
	float32 maxImpulse;
	int32 dependentImpulse;

	Jsp j;
	Bsp b;
	float32 d;

	float32 fGradient;
	float32 searchDir;
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

struct NNCGSolverStepInfo
{
	NNCGSolverStepInfo()
	{
		curGradientMagSqr = 0.0f;
		prevGradientMagSqr = 0.0f;
	}

	float32 curGradientMagSqr;
	float32 prevGradientMagSqr;
};

struct NNCGSolverData
{
	StackAllocator* stackAllocator;
	TimeStep timeStep;
	NNCGSolverStepInfo prevStepInfo;

	int32 rigidbodyCount;
	int32 contactCount;
	int32 jointCount;

	Vector3* positions;
	Quaternion* orientations;
	Vector3* linearVelocities;
	Vector3* angularVelocities;
	Contact** contactsInIsland;
	Joint** jointsInIsland;
};

class NNCGSolver
{
public:
	NNCGSolver(const NNCGSolverData& data);
	~NNCGSolver();

	void solveVelocityConstraints();
	void saveImpulse();
	void applyeDelta();
	bool solvePositionConstraints();

	NNCGSolverStepInfo getStepInfo() const
	{
		NNCGSolverStepInfo ret;
		ret.curGradientMagSqr = curGradientMagSqr;
		ret.prevGradientMagSqr = prevGradientMagSqr;
		return ret;
	}

private:
	struct ContactVCCacheRef;

	StackAllocator* stackAllocator;

	Contact** contacts;

	int32 jointCount;
	Joint** joints;

	Vector3* positions;
	Quaternion* orientations;
	Vector3* linearVelocities;
	Vector3* angularVelocities;

	int32 jointVCCount;
	int32 contactVCCount;
	int32 velConstraintsCount;
	VelocityConstraint* velocityConstraints;
	ContactVCCacheRef* contactVCCacheRefs;

	int32 contactPCCount;
	ContactPositionConstraint* positionConstraints;

	int32 rigidbodyCount;
	Vector3* dLinearV;
	Vector3* dAngularV;

	float32 curGradientMagSqr;
	float32 prevGradientMagSqr;


	void PGS();
};