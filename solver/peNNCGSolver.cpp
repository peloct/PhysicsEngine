#include"peNNCGSolver.h"
#include"../memory/peStackAllocator.h"
#include"../collision/peContact.h"
#include"../peFixture.h"
#include"../peRigidbody.h"

#define BOUNCE_THRESHOLD -1.0f
#define BAUMGARTE 0.2f
#define LINEAR_SLOP 0.005f

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

struct NNCGSolver::ContactVelocityConstraint
{
	int32 contactIndex;
	int32 pointIndex;
	int32 dirIndex;

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

struct NNCGSolver::ContactPositionConstraint
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

NNCGSolver::NNCGSolver(const NNCGSolverDef& def)
{
	stackAllocator = def.stackAllocator;

	contactCount = def.contactCount;
	contacts = def.contactsInIsland;

	positions = def.positions;
	orientations = def.orientations;
	linearVelocities = def.linearVelocities;
	angularVelocities = def.angularVelocities;

	velConstraintsCount = 0;
	for (int i = 0; i < contactCount; ++i)
		velConstraintsCount += contacts[i]->contactPointCount;
	velConstraintsCount *= 3;

	velocityConstraints = (ContactVelocityConstraint*)stackAllocator->allocate(velConstraintsCount * sizeof(ContactVelocityConstraint));

	posConstraintsCount = contactCount;
	positionConstraints = (ContactPositionConstraint*)stackAllocator->allocate(posConstraintsCount * sizeof(ContactPositionConstraint));

	rigidbodyCount = def.rigidbodyCount;
	dLinearV = (Vector3*)stackAllocator->allocate(rigidbodyCount * sizeof(Vector3));
	dAngularV = (Vector3*)stackAllocator->allocate(rigidbodyCount * sizeof(Vector3));

	prevGradientMagSqr = def.prevStepInfo.prevGradientMagSqr;
	curGradientMagSqr = def.prevStepInfo.curGradientMagSqr;

	for (int i = 0; i < rigidbodyCount; ++i)
	{
		dLinearV[i].setZero();
		dAngularV[i].setZero();
	}

	int vcIndex = 0;
	for (int i = 0; i < contactCount; ++i)
	{
		Contact* contact = contacts[i];

		const Rigidbody* rigidbodyA = contact->fixtureA->getRigidbody();
		const Rigidbody* rigidbodyB = contact->fixtureB->getRigidbody();

		int32 indexA = rigidbodyA->islandID;
		int32 indexB = rigidbodyB->islandID;

		Vector3 pA = positions[indexA];
		Vector3 pB = positions[indexB];
		Vector3 vA = linearVelocities[indexA];
		Vector3 vB = linearVelocities[indexB];
		Vector3 wA = angularVelocities[indexA];
		Vector3 wB = angularVelocities[indexB];

		float32 invMassA = rigidbodyA->invMass;
		float32 invMassB = rigidbodyB->invMass;

		const Matrix3x3& invWorldIA = rigidbodyA->invWorldInertiaTensor;
		const Matrix3x3& invWorldIB = rigidbodyB->invWorldInertiaTensor;

		Matrix4x4 bodyLocalToWorldA = contact->fixtureA->getBodyLocalToWorld();
		Matrix4x4 bodyLocalToWorldB = contact->fixtureB->getBodyLocalToWorld();

		Vector3 dir[3];
		dir[0] = bodyLocalToWorldA.transformDirection(contact->localNormal);
		dir[1] = bodyLocalToWorldA.transformDirection(contact->localTangent1);
		dir[2] = bodyLocalToWorldA.transformDirection(contact->localTangent2);

		ContactPositionConstraint* pc = positionConstraints + i;

		pc->faceOwner = contact->contactFaceOwner;
		pc->indexA = rigidbodyA->islandID;
		pc->indexB = rigidbodyB->islandID;
		pc->invMassA = rigidbodyA->invMass;
		pc->invMassB = rigidbodyB->invMass;
		pc->invIA = rigidbodyA->invInertiaTensor;
		pc->invIB = rigidbodyB->invInertiaTensor;
		pc->localCenterA = rigidbodyA->centerOfMass;
		pc->localCenterB = rigidbodyB->centerOfMass;
		pc->localNormal = contact->localNormal;
		pc->localPlanePoint = contact->localPlanePoint;
		pc->contactPointCount = contact->contactPointCount;

		for (int j = 0; j < contact->contactPointCount; ++j)
		{
			ContactPoint* cp = contact->contactPoints + j;

			pc->contactPoints[j] = cp->localPoint;

			float32 cachedImpulse[3];
			cachedImpulse[0] = cp->normalImpulse * def.timeStep.ratio;
			cachedImpulse[1] = cp->tangentImpulse.x * def.timeStep.ratio;
			cachedImpulse[2] = cp->tangentImpulse.y * def.timeStep.ratio;
			float32 fGradient[3] = { cp->fGradient.x, cp->fGradient.y, cp->fGradient.z };
			float32 direction[3] = { cp->direction.x, cp->direction.y, cp->direction.z };

			Vector3 worldPoint = bodyLocalToWorldB.transformPoint(cp->localPoint);

			Vector3 rA = worldPoint - pA;
			Vector3 rB = worldPoint - pB;

			for (int k = 0; k < 3; ++k)
			{
				ContactVelocityConstraint* vc = velocityConstraints + vcIndex;

				vc->contactIndex = i;
				vc->pointIndex = j;
				vc->dirIndex = k;

				Vector3 dirA = -dir[k];
				Vector3 dirB = dir[k];

				Jsp& j = vc->j;

				j.linearA = dirA;
				j.angularA = Vector3::cross(rA, dirA);
				j.linearB = dirB;
				j.angularB = Vector3::cross(rB, dirB);

				Bsp& b = vc->b;

				b.linearA = invMassA * j.linearA;
				b.angularA = invWorldIA * j.angularA;
				b.linearB = invMassB * j.linearB;
				b.angularB = invWorldIB * j.angularB;

				float32 impulse = cachedImpulse[k];

				vc->impulse = impulse;

				dLinearV[indexA] += impulse * b.linearA;
				dAngularV[indexA] += impulse * b.angularA;
				dLinearV[indexB] += impulse * b.linearB;
				dAngularV[indexB] += impulse * b.angularB;

				vc->d =
					Vector3::dot(j.linearA, b.linearA)
					+ Vector3::dot(j.angularA, b.angularA)
					+ Vector3::dot(j.linearB, b.linearB)
					+ Vector3::dot(j.angularB, b.angularB);

				vc->indexA = indexA;
				vc->indexB = indexB;

				Vector3 relVel = vB + Vector3::cross(wB, rB) - vA - Vector3::cross(wA, rA);
				float32 relVelToDir = Vector3::dot(relVel, dirB);

				vc->dependentImpulse = -1;
				vc->minImpulse = -MaxFloat32;
				vc->maxImpulse = MaxFloat32;

				if (k == 0)
				{
					if (relVelToDir < BOUNCE_THRESHOLD)
						vc->desiredDelV = -relVelToDir * (1.0f + contact->restitution);
					else
						vc->desiredDelV = -relVelToDir;
					vc->minImpulse = 0.0f;
				}
				else
				{
					vc->desiredDelV = -relVelToDir;
					vc->dependentImpulse = vcIndex - k;
					vc->minImpulse = -contact->friction;
					vc->maxImpulse = contact->friction;
				}

				vc->fGradient = fGradient[k];
				vc->searchDir = direction[k];

				++vcIndex;
			}
		}
	}
}

NNCGSolver::~NNCGSolver()
{
	stackAllocator->free(dAngularV);
	stackAllocator->free(dLinearV);
	stackAllocator->free(positionConstraints);
	stackAllocator->free(velocityConstraints);
}

void NNCGSolver::initVelocityConstraints()
{
}

void NNCGSolver::warmStart()
{
}

void NNCGSolver::PGS()
{
	for (int i = 0; i < velConstraintsCount; ++i)
	{
		ContactVelocityConstraint* vc = velocityConstraints + i;

		int32 indexA = vc->indexA;
		int32 indexB = vc->indexB;

		const Bsp& b = vc->b;

		float32 delV =
			Vector3::dot(vc->j.linearA, dLinearV[indexA])
			+ Vector3::dot(vc->j.angularA, dAngularV[indexA])
			+ Vector3::dot(vc->j.linearB, dLinearV[indexB])
			+ Vector3::dot(vc->j.angularB, dAngularV[indexB]);

		float32 dImpulse = (vc->desiredDelV - delV) / vc->d;
		float32 impulse0 = vc->impulse;

		float32 minImpulse = vc->minImpulse;
		float32 maxImpulse = vc->maxImpulse;

		if (vc->dependentImpulse >= 0)
		{
			float32 impulse = velocityConstraints[vc->dependentImpulse].impulse;
			minImpulse = impulse * minImpulse;
			maxImpulse = impulse * maxImpulse;
		}

		float32 impulse = peClamp(impulse0 + dImpulse, minImpulse, maxImpulse);
		dImpulse = impulse - impulse0;
		vc->impulse = impulse;
		vc->fGradient = -dImpulse;
		curGradientMagSqr += dImpulse * dImpulse;

		dLinearV[indexA] += dImpulse * b.linearA;
		dAngularV[indexA] += dImpulse * b.angularA;
		dLinearV[indexB] += dImpulse * b.linearB;
		dAngularV[indexB] += dImpulse * b.angularB;
	}
}

void NNCGSolver::solveVelocityConstraints()
{
	hit = false;

	if (prevGradientMagSqr > 0.0f)
	{
		float32 beta = curGradientMagSqr / prevGradientMagSqr;

		if (beta > 1)
		{
			for (int i = 0; i < velConstraintsCount; ++i)
			{
				ContactVelocityConstraint* vc = velocityConstraints + i;
				vc->searchDir = 0;
			}
		}
		else
		{
			hit = true;

			for (int i = 0; i < velConstraintsCount; ++i)
			{
				ContactVelocityConstraint* vc = velocityConstraints + i;
				float32 searchImpulse = beta * vc->searchDir;
				
				const Bsp& b = vc->b;
				int32 indexA = vc->indexA;
				int32 indexB = vc->indexB;

				dLinearV[indexA] += searchImpulse * b.linearA;
				dAngularV[indexA] += searchImpulse * b.angularA;
				dLinearV[indexB] += searchImpulse * b.linearB;
				dAngularV[indexB] += searchImpulse * b.angularB;

				vc->searchDir = searchImpulse - vc->fGradient;
			}
		}
	}
	else
	{
		for (int i = 0; i < velConstraintsCount; ++i)
		{
			ContactVelocityConstraint* vc = velocityConstraints + i;
			vc->searchDir = -vc->fGradient;
		}
	}

	prevGradientMagSqr = curGradientMagSqr;
	curGradientMagSqr = 0.0f;

	PGS();
}

void NNCGSolver::saveImpulse()
{
	for (int i = 0; i < velConstraintsCount; ++i)
	{
		ContactVelocityConstraint* vc = velocityConstraints + i;
		Contact* contact = contacts[vc->contactIndex];

		ContactPoint* cp = contact->contactPoints + vc->pointIndex;
		
		if (vc->dirIndex == 0)
		{
			cp->normalImpulse = vc->impulse;
			cp->direction.x = vc->searchDir;
			cp->fGradient.x = vc->fGradient;
		}
		else if (vc->dirIndex == 1)
		{
			cp->tangentImpulse.x = vc->impulse;
			cp->direction.y = vc->searchDir;
			cp->fGradient.y = vc->fGradient;
		}
		else
		{
			cp->tangentImpulse.y = vc->impulse;
			cp->direction.z = vc->searchDir;
			cp->fGradient.z = vc->fGradient;
		}
	}
}

void NNCGSolver::applyeDelta()
{
	for (int i = 0; i < rigidbodyCount; ++i)
	{
		linearVelocities[i] += dLinearV[i];
		angularVelocities[i] += dAngularV[i];
	}
}

bool NNCGSolver::solvePositionConstraints()
{
	float32 minSeparation = 0.0f;

	for (int i = 0; i < posConstraintsCount; ++i)
	{
		ContactPositionConstraint* pc = positionConstraints + i;

		int32 refIndex;
		int32 incIndex;
		Vector3 refLocalCenter;
		Vector3 incLocalCenter;
		float32 refInvMass;
		float32 incInvMass;
		Matrix3x3 refInvI;
		Matrix3x3 incInvI;

		if (pc->faceOwner == ContactFaceOwner::fixtureA)
		{
			refIndex = pc->indexA;
			incIndex = pc->indexB;
			refLocalCenter = pc->localCenterA;
			incLocalCenter = pc->localCenterB;
			refInvMass = pc->invMassA;
			incInvMass = pc->invMassB;
			refInvI = pc->invIA;
			incInvI = pc->invIB;
		}
		else
		{
			refIndex = pc->indexB;
			incIndex = pc->indexA;
			refLocalCenter = pc->localCenterB;
			incLocalCenter = pc->localCenterA;
			refInvMass = pc->invMassB;
			incInvMass = pc->invMassA;
			refInvI = pc->invIB;
			incInvI = pc->invIA;
		}

		Vector3 pRef = positions[refIndex];
		Quaternion qRef = orientations[refIndex];
		Vector3 pInc = positions[incIndex];
		Quaternion qInc = orientations[incIndex];

		for (int j = 0; j < pc->contactPointCount; ++j)
		{
			Vector3 bpRef = pRef - (qRef * refLocalCenter);
			Vector3 bpInc = pInc - (qInc * incLocalCenter);

			Vector3 clipPoint = bpInc + qInc * pc->contactPoints[j];
			Vector3 planePoint = bpRef + qRef * pc->localPlanePoint;
			Vector3 normal = qRef * pc->localNormal;

			float32 separation = Vector3::dot(clipPoint - planePoint, normal);

			minSeparation = peMinf(minSeparation, separation);

			Matrix3x3 orientationRef;
			qRef.getOrientationMatrix(&orientationRef);
			Matrix3x3 orientationInc;
			qInc.getOrientationMatrix(&orientationInc);

			Vector3 rRef = clipPoint - pRef;
			Vector3 torqueImpulseRef = Vector3::cross(rRef, normal);
			Vector3 dqRef = orientationRef * (refInvI * orientationRef.transposeMuliply(torqueImpulseRef));

			Vector3 rInc = clipPoint - pInc;
			Vector3 torqueImpulseInc = Vector3::cross(rInc, normal);
			Vector3 dqInc = orientationInc * (incInvI * orientationInc.transposeMuliply(torqueImpulseInc));

			float32 refDeltaPerLambda = refInvMass + Vector3::dot(Vector3::cross(dqRef, rRef), normal);
			float32 incDeltaPerLambda = incInvMass + Vector3::dot(Vector3::cross(dqInc, rInc), normal);

			float32 c = peClamp(BAUMGARTE * (separation + LINEAR_SLOP), -0.2f, 0.0f);

			float32 lambda = -c / (refDeltaPerLambda + incDeltaPerLambda);

			pRef += refInvMass * -lambda * normal;
			pInc += incInvMass * lambda * normal;
			qRef.addScaledVector(-lambda * dqRef, 1);
			qInc.addScaledVector(lambda * dqInc, 1);

			qRef.normailize();
			qInc.normailize();
		}

		positions[refIndex] = pRef;
		orientations[refIndex] = qRef;
		positions[incIndex] = pInc;
		orientations[incIndex] = qInc;
	}

	return minSeparation > -1.5 * LINEAR_SLOP;
}