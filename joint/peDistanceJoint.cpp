#include"peDistanceJoint.h"
#include"..//peRigidbody.h"
#include"..//solver/peNNCGSolver.h"

#define INF 100000000.0f
#define IGNORE_TOL  0.0001f

DistanceJoint::DistanceJoint(const DistanceJointDef& def) : Joint(def.jointType, def.bodyA, def.bodyB)
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

	damping = def.damping;
	spring = def.spring;
	targetDistance = def.targetDistance != -1.0f? def.targetDistance : (anchorPosA - anchorPosB).magnitude();
	isMax = def.isMax;
	isMin = def.isMin;
	restitution = def.restitution;

	localAnchorA -= bodyA->centerOfMass;
	if (bodyB)
		localAnchorB -= bodyB->centerOfMass;
}

int32 DistanceJoint::getVelConstraintCount() const
{
	return 1;
}

void DistanceJoint::createVelConstraints(const NNCGSolverData& solverData, VelocityConstraint* velocityConstraints, Vector3* dLinearV, Vector3* dAngularV)
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

	if (curDistance < IGNORE_TOL)
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

	float32 diffLength = curDistance - targetDistance;
	if (diffLength < 0)
		diffLength *= -1;

	float32 dampingF;
	if (curDistance < targetDistance)
		dampingF = damping * peAbsf(relVelToDir) * (relVelToDir > 0 ? -1 : 1);
	else
		dampingF = damping * peAbsf(relVelToDir) * (relVelToDir > 0 ? 1 : -1);

	float32 springImpulseAbs = solverData.timeStep.dt * (dampingF + spring * diffLength);

	velocityConstraints->doSolve = true;
	velocityConstraints->cacheRef = static_cast<Joint*>(this);
	velocityConstraints->indexA = indexA;
	velocityConstraints->indexB = indexB;
	velocityConstraints->dependentImpulse = NULL_ID;
	velocityConstraints->fGradient = fGradient;
	velocityConstraints->searchDir = searchDir;
	velocityConstraints->impulse = impulse;

	if (isMin && isMax)
	{
		velocityConstraints->desiredDelV = -relVelToDir;
		velocityConstraints->maxImpulse = MaxFloat32;
		velocityConstraints->minImpulse = -MaxFloat32;
	}
	else if (isMin)
	{
		if (curDistance <= targetDistance)
		{
			if (relVelToDir < BOUNCE_THRESHOLD)
				velocityConstraints->desiredDelV = -(1.0f + restitution) * relVelToDir;
			else
				velocityConstraints->desiredDelV = -relVelToDir;
			velocityConstraints->maxImpulse = MaxFloat32;
			velocityConstraints->minImpulse = 0.0f;
		}
		else
		{
			velocityConstraints->desiredDelV = INF;
			velocityConstraints->maxImpulse = springImpulseAbs;
			velocityConstraints->minImpulse = 0.0f;
		}
	}
	else if (isMax)
	{
		if (curDistance >= targetDistance)
		{
			if (relVelToDir > -BOUNCE_THRESHOLD)
				velocityConstraints->desiredDelV = -(1.0f + restitution) * relVelToDir;
			else
				velocityConstraints->desiredDelV = -relVelToDir;
			velocityConstraints->maxImpulse = 0.0f;
			velocityConstraints->minImpulse = -MaxFloat32;
		}
		else
		{
			velocityConstraints->desiredDelV = -INF;
			velocityConstraints->maxImpulse = 0.0f;
			velocityConstraints->minImpulse = -springImpulseAbs;
		}
	}
	else
	{
		if (curDistance >= targetDistance)
			velocityConstraints->desiredDelV = -INF;
		else
			velocityConstraints->desiredDelV = INF;
		velocityConstraints->maxImpulse = springImpulseAbs;
		velocityConstraints->minImpulse = -springImpulseAbs;
	}

	dLinearV[indexA] += impulse * b.linearA;
	dAngularV[indexA] += impulse * b.angularA;
	dLinearV[indexB] += impulse * b.linearB;
	dAngularV[indexB] += impulse * b.angularB;
}

void DistanceJoint::saveImpulse(VelocityConstraint* velocityConstraints)
{
	impulse = velocityConstraints->impulse;
	fGradient = velocityConstraints->fGradient;
	searchDir = velocityConstraints->searchDir;
}

bool DistanceJoint::solvePosConstraints(Vector3* positions, Quaternion* orientations) const
{
	if (!isMin && !isMax)
		return true;

	int32 indexA = bodyA->islandID;
	int32 indexB = bodyB ? bodyB->islandID : NULL_ID;

	Vector3 posA = positions[indexA];
	Vector3 posB;
	if (indexB != NULL_ID)
		posB = positions[indexB];
	else
		posB.setZero();

	Quaternion orientationA = orientations[indexA];
	Quaternion orientationB = indexB != NULL_ID ? orientations[indexB] : Quaternion();

	Vector3 anchorRelPosA = orientationA * localAnchorA;
	Vector3 anchorRelPosB = orientationB * localAnchorB;

	Vector3 anchorPosA = posA + anchorRelPosA;
	Vector3 anchorPosB = posB + anchorRelPosB;

	Vector3 dir = anchorPosB - anchorPosA;
	float32 curDistance = dir.magnitude();
	float32 correction = 0.0f;

	if (curDistance < IGNORE_TOL)
		return true;

	if (isMin && curDistance < targetDistance)
		correction = -peClamp(BAUMGARTE * (curDistance - targetDistance + LINEAR_SLOP), -0.2f, 0.0f);
	else if (isMax && curDistance > targetDistance)
		correction = peClamp(BAUMGARTE * (targetDistance - curDistance + LINEAR_SLOP), -0.2f, 0.0f);
	else
		return true;

	dir /= curDistance;

	float32 invMassA = bodyA->invMass;
	float32 invMassB = bodyB ? bodyB->invMass : 0.0f;

	Matrix3x3 zero;
	const Matrix3x3& invWorldIA = bodyA->invWorldInertiaTensor;
	const Matrix3x3& invWorldIB = bodyB ? bodyB->invWorldInertiaTensor : zero;

	Matrix3x3 orientationMatA;
	orientationA.getOrientationMatrix(&orientationMatA);
	Matrix3x3 orientationMatB;
	orientationB.getOrientationMatrix(&orientationMatB);

	Vector3 torqueImpulseA = Vector3::cross(anchorRelPosA, dir);
	Vector3 dqRef = orientationMatA * (invWorldIA * orientationMatA.transposeMuliply(torqueImpulseA));

	Vector3 torqueImpulseB = Vector3::cross(anchorRelPosB, dir);
	Vector3 dqInc = orientationMatB * (invWorldIB * orientationMatB.transposeMuliply(torqueImpulseB));

	float32 deltaPerLambdaA = invMassA + Vector3::dot(Vector3::cross(dqRef, anchorRelPosA), dir);
	float32 deltaPerLambdaB = invMassB + Vector3::dot(Vector3::cross(dqInc, anchorRelPosB), dir);

	float32 lambda = correction / (deltaPerLambdaA + deltaPerLambdaB);

	posA += invMassA * -lambda * dir;
	posB += invMassB * lambda * dir;
	orientationA.addScaledVector(-lambda * dqRef, 1);
	orientationB.addScaledVector(lambda * dqInc, 1);

	orientationA.normailize();
	orientationB.normailize();

	positions[indexA] = posA;
	orientations[indexA] = orientationA;

	if (indexB != NULL_ID)
	{
		positions[indexB] = posB;
		orientations[indexB] = orientationB;
	}

	return correction < 1.5f * LINEAR_SLOP;
}