#include"peSpringJoint.h"
#include"..//peRigidbody.h"
#include"..//solver/peNNCGSolver.h"

SpringJoint::SpringJoint(const SpringJointDef& def) : Joint(def.jointType, def.bodyA, def.bodyB)
{
	localAnchorA = def.localAnchorA;
	localAnchorB = def.localAnchorB;

	fGradient = 0.0f;
	searchDir = 0.0f;
	impulse = 0.0f;

	Matrix4x4 localToWorldA;
	bodyA->getLocalToWorld(&localToWorldA);
	Matrix4x4 localToWorldB;
	if (bodyB)
		bodyB->getLocalToWorld(&localToWorldB);
	else
		localToWorldB.identity();

	Vector3 anchorPosA = localToWorldA.transformPoint(localAnchorA);
	Vector3 anchorPosB = localToWorldB.transformPoint(localAnchorB);

	spring = def.spring;
	referenceDistance = (anchorPosA - anchorPosB).magnitude();

	localAnchorA -= bodyA->centerOfMass;
	if (bodyB)
		localAnchorB -= bodyB->centerOfMass;
}

int32 SpringJoint::getVelConstraintCount() const
{
	return 1;
}

void SpringJoint::createVelConstraints(const NNCGSolverData& solverData, VelocityConstraint* velocityConstraints, Vector3* dLinearV, Vector3* dAngularV)
{
	int32 indexA = bodyA->islandID;
	int32 indexB = bodyB ? bodyB->islandID : NULL_ID;

	Vector3 posA = solverData.positions[indexA];
	Vector3 posB;
	if (indexB != NULL_ID)
		posB = solverData.positions[indexB];
	else
		posB.setZero();

	Vector3 anchorRelPosA = solverData.orientations[indexA] * localAnchorA;
	Vector3 anchorRelPosB = indexB != NULL_ID ? solverData.orientations[indexB] * localAnchorB : localAnchorB;

	Vector3 anchorPosA = posA + anchorRelPosA;
	Vector3 anchorPosB = posB + anchorRelPosB;

	Vector3 dir = anchorPosB - anchorPosA;
	float32 curDistance = dir.magnitude();

	if (curDistance < 0.0001f)
	{
		velocityConstraints->doSolve = false;
		velocityConstraints->impulse = 0.0f;
		velocityConstraints->searchDir = 0.0f;
		velocityConstraints->fGradient = 0.0f;
		return;
	}

	dir /= curDistance;

	Vector3 vA = solverData.linearVelocities[indexA];
	Vector3 vB = indexB != NULL_ID ? solverData.linearVelocities[indexB] : Vector3();
	Vector3 wA = solverData.angularVelocities[indexA];
	Vector3 wB = indexB != NULL_ID ? solverData.angularVelocities[indexB] : Vector3();

	Jsp& j = velocityConstraints->j;
	Bsp& b = velocityConstraints->b;

	Vector3 dirA = -dir;
	Vector3 dirB = dir;

	j.linearA = dirA;
	j.angularA = Vector3::cross(anchorRelPosA, dirA);
	j.linearB = dirB;
	j.angularB = Vector3::cross(anchorRelPosB, dirB);

	float32 invMassA = bodyA->invMass;
	float32 invMassB = bodyB ? bodyB->invMass : 0.0f;

	Matrix3x3 zero;
	const Matrix3x3& invWorldIA = bodyA->invWorldInertiaTensor;
	const Matrix3x3& invWorldIB = bodyB ? bodyB->invWorldInertiaTensor : zero;

	b.linearA = invMassA * j.linearA;
	b.angularA = invWorldIA * j.angularA;
	b.linearB = invMassB * j.linearB;
	b.angularB = invWorldIB * j.angularB;

	velocityConstraints->d =
		Vector3::dot(j.linearA, b.linearA)
		+ Vector3::dot(j.angularA, b.angularA)
		+ Vector3::dot(j.linearB, b.linearB)
		+ Vector3::dot(j.angularB, b.angularB);

	Vector3 relVel = vB + Vector3::cross(wB, anchorRelPosB) - vA - Vector3::cross(wA, anchorRelPosA);
	float32 relVelToDir = Vector3::dot(relVel, dirB);

	velocityConstraints->dependentImpulse = NULL_ID;
	velocityConstraints->desiredDelV = - solverData.timeStep.dt * spring * (curDistance - referenceDistance);
	velocityConstraints->doSolve = true;
	velocityConstraints->cacheRef = static_cast<Joint*>(this);
	velocityConstraints->indexA = indexA;
	velocityConstraints->indexB = indexB;
	velocityConstraints->maxImpulse = MaxFloat32;
	velocityConstraints->minImpulse = -MaxFloat32;
	velocityConstraints->fGradient = fGradient;
	velocityConstraints->searchDir = searchDir;
	velocityConstraints->impulse = impulse;

	dLinearV[indexA] += impulse * b.linearA;
	dAngularV[indexA] += impulse * b.angularA;
	dLinearV[indexB] += impulse * b.linearB;
	dAngularV[indexB] += impulse * b.angularB;
}

void SpringJoint::saveImpulse(VelocityConstraint* velocityConstraints)
{
	impulse = velocityConstraints->impulse;
	fGradient = velocityConstraints->fGradient;
	searchDir = velocityConstraints->searchDir;
}

bool SpringJoint::solvePosConstraints(Vector3* positions, Quaternion* orientations) const
{
	return true;
}