#include"peContactSolver.h"
#include"../memory/peStackAllocator.h"
#include"../collision/peContact.h"
#include"../peFixture.h"
#include"../peRigidbody.h"

ContactSolver::ContactSolver(const ContactSolverDef& def)
{
	stackAllocator = def.stackAllocator;
	contacts = def.contactsInIsland;

	positions = def.positions;
	orientations = def.orientations;
	linearVelocities = def.linearVelocities;
	angularVelocities = def.angularVelocities;

	constraintsCount = def.contactCount;
	velocityConstraints = (ContactVelocityConstraint*)stackAllocator->allocate(constraintsCount * sizeof(ContactVelocityConstraint));
	positionConstraints = (ContactPositionConstraint*)stackAllocator->allocate(constraintsCount * sizeof(ContactPositionConstraint));

	for (int i = 0; i < constraintsCount; ++i)
	{
		Contact* contact = def.contactsInIsland[i];

		const Rigidbody* rigidbodyA = contact->fixtureA->getRigidbody();
		const Rigidbody* rigidbodyB = contact->fixtureB->getRigidbody();

		ContactVelocityConstraint* vc = velocityConstraints + i;
		vc->contactIndex = contact->islandID;
		vc->indexA = rigidbodyA->islandID;
		vc->indexB = rigidbodyB->islandID;
		vc->friction = contact->friction;
		vc->restitution = contact->restitution;
		vc->invMassA = rigidbodyA->invMass;
		vc->invMassB = rigidbodyB->invMass;
		vc->invWorldIA = rigidbodyA->invWorldInertiaTensor;
		vc->invWorldIB = rigidbodyB->invWorldInertiaTensor;
		vc->contactPointCount = contact->contactPointCount;

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
			ContactVelocityConstraintPoint* vcp = vc->contactPoints + j;
			ContactPoint* cp = contact->contactPoints + j;

			vcp->normalImpulse = def.timeStep.ratio * cp->normalImpulse;
			vcp->tangentImpulse = def.timeStep.ratio * cp->tangentImpulse;
			
			vcp->normalMass = 0;
			vcp->tangentMass.setZero();
			vcp->rA.setZero();
			vcp->rB.setZero();
			vcp->velocityBias = 0;

			pc->contactPoints[j] = cp->localPoint;
		}
	}
}

ContactSolver::~ContactSolver()
{
	stackAllocator->free(positionConstraints);
	stackAllocator->free(velocityConstraints);
}

void ContactSolver::initVelocityConstraints()
{
	Matrix3x3 identity;
	identity.identity();

	for (int i = 0; i < constraintsCount; ++i)
	{
		ContactVelocityConstraint* vc = velocityConstraints + i;
		Contact* contact = contacts[vc->contactIndex];
		const Rigidbody* rigidbodyA = contact->fixtureA->getRigidbody();
		const Rigidbody* rigidbodyB = contact->fixtureB->getRigidbody();
		Vector3 pA = positions[rigidbodyA->islandID];
		Vector3 pB = positions[rigidbodyB->islandID];
		Vector3 vA = linearVelocities[rigidbodyA->islandID];
		Vector3 vB = linearVelocities[rigidbodyB->islandID];
		Vector3 wA = angularVelocities[rigidbodyA->islandID];
		Vector3 wB = angularVelocities[rigidbodyB->islandID];

		Matrix4x4 bodyLocalToWorldA = contact->fixtureA->getBodyLocalToWorld();
		Matrix4x4 bodyLocalToWorldB = contact->fixtureB->getBodyLocalToWorld();
		
		vc->normal = bodyLocalToWorldA.transformDirection(contact->localNormal);
		vc->tangent1 = bodyLocalToWorldA.transformDirection(contact->localTangent1);
		vc->tangent2 = bodyLocalToWorldA.transformDirection(contact->localTangent2);
		
		for (int j = 0; j < vc->contactPointCount; ++j)
		{
			ContactVelocityConstraintPoint* vcp = vc->contactPoints + j;
			ContactPoint* cp = contact->contactPoints + j;
			Vector3 worldPoint = bodyLocalToWorldB.transformPoint(cp->localPoint);

			Vector3 rA = worldPoint - pA;
			Vector3 rB = worldPoint - pB;

			vcp->rA = rA;
			vcp->rB = rB;
			
			Matrix3x3 rAc;
			rA.getCrossMatrix(&rAc);
			Matrix3x3 rBc;
			rB.getCrossMatrix(&rBc);

			Matrix3x3 m = identity * (vc->invMassA + vc->invMassB) - rAc * vc->invWorldIA * rAc - rBc * vc->invWorldIB * rBc;

			vcp->normalMass = 1 / Vector3::dot(m * vc->normal, vc->normal);
			
			Matrix2x2 tangentMass;
			Vector3 mt1 = m * vc->tangent1;
			Vector3 mt2 = m * vc->tangent2;
			tangentMass.m00 = Vector3::dot(mt1, vc->tangent1);
			tangentMass.m01 = Vector3::dot(mt2, vc->tangent1);
			tangentMass.m10 = Vector3::dot(mt1, vc->tangent2);
			tangentMass.m11 = Vector3::dot(mt2, vc->tangent2);
			tangentMass.inverse();

			vcp->tangentMass = tangentMass;

			Vector3 relVel = vB + Vector3::cross(wB, rB) - vA - Vector3::cross(wA, rA);
			float32 normalRelVel = Vector3::dot(relVel, vc->normal);

			if (normalRelVel < BOUNCE_THRESHOLD)
				vcp->velocityBias = -normalRelVel * contact->restitution;
			else
				vcp->velocityBias = 0;
		}
	}
}

void ContactSolver::warmStart()
{
	for (int i = 0; i < constraintsCount; ++i)
	{
		ContactVelocityConstraint* vc = velocityConstraints + i;

		for (int j = 0; j < vc->contactPointCount; ++j)
		{
			ContactVelocityConstraintPoint* vcp = vc->contactPoints + j;
			
			Vector3 vA = linearVelocities[vc->indexA];
			Vector3 vB = linearVelocities[vc->indexB];
			Vector3 wA = angularVelocities[vc->indexA];
			Vector3 wB = angularVelocities[vc->indexB];

			Vector3 rA = vcp->rA;
			Vector3 rB = vcp->rB;

			Vector3 impulse =
				vcp->normalImpulse * vc->normal
				+ vcp->tangentImpulse.x * vc->tangent1
				+ vcp->tangentImpulse.y * vc->tangent2;

			vA -= vc->invMassA * impulse;
			vB += vc->invMassB * impulse;
			wA -= vc->invWorldIA * Vector3::cross(rA, impulse);
			wB += vc->invWorldIB * Vector3::cross(rB, impulse);

			linearVelocities[vc->indexA] = vA;
			linearVelocities[vc->indexB] = vB;
			angularVelocities[vc->indexA] = wA;
			angularVelocities[vc->indexB] = wB;
		}
	}
}

void ContactSolver::solveVelocityConstraints()
{
	for (int i = 0; i < constraintsCount; ++i)
	{
		ContactVelocityConstraint* vc = velocityConstraints + i;

		// tangent
		for (int j = 0; j < vc->contactPointCount; ++j)
		{
			ContactVelocityConstraintPoint* vcp = vc->contactPoints + j;
			
			Vector3 rA = vcp->rA;
			Vector3 rB = vcp->rB;
			Vector3 vA = linearVelocities[vc->indexA];
			Vector3 vB = linearVelocities[vc->indexB];
			Vector3 wA = angularVelocities[vc->indexA];
			Vector3 wB = angularVelocities[vc->indexB];

			Vector3 relVel = vB + Vector3::cross(wB, rB) - vA - Vector3::cross(wA, rA);
			Vector2 tanRelVel;
			tanRelVel.x = Vector3::dot(relVel, vc->tangent1);
			tanRelVel.y = Vector3::dot(relVel, vc->tangent2);
			Vector2 lambda = vcp->tangentMass * (-tanRelVel);

			Vector2 tangentImpulse = vcp->tangentImpulse + lambda;
			float32 mag = tangentImpulse.magnitude();
			float32 maxTangentImpulseRange = vcp->normalImpulse * vc->friction;
			float32 newMag = peMinf(mag, maxTangentImpulseRange);

			if (mag > newMag)
				tangentImpulse = tangentImpulse * (newMag / mag);

			lambda = tangentImpulse - vcp->tangentImpulse;
			vcp->tangentImpulse = tangentImpulse;

			Vector3 tangentImpulseW = vc->tangent1 * lambda.x + vc->tangent2 * lambda.y;

			vA -= vc->invMassA * tangentImpulseW;
			vB += vc->invMassB * tangentImpulseW;
			wA -= vc->invWorldIA * Vector3::cross(rA, tangentImpulseW);
			wB += vc->invWorldIB * Vector3::cross(rB, tangentImpulseW);

			linearVelocities[vc->indexA] = vA;
			linearVelocities[vc->indexB] = vB;
			angularVelocities[vc->indexA] = wA;
			angularVelocities[vc->indexB] = wB;
		}

		// normal
		for (int j = 0; j < vc->contactPointCount; ++j)
		{
			ContactVelocityConstraintPoint* vcp = vc->contactPoints + j;

			Vector3 rA = vcp->rA;
			Vector3 rB = vcp->rB;
			Vector3 vA = linearVelocities[vc->indexA];
			Vector3 vB = linearVelocities[vc->indexB];
			Vector3 wA = angularVelocities[vc->indexA];
			Vector3 wB = angularVelocities[vc->indexB];

			Vector3 relVel = vB + Vector3::cross(wB, rB) - vA - Vector3::cross(wA, rA);
			float32 normalRelVel = Vector3::dot(relVel, vc->normal);
			float32 lambda = vcp->normalMass * (vcp->velocityBias - normalRelVel);

			float32 normalImpulse = vcp->normalImpulse + lambda;
			normalImpulse = peMaxf(normalImpulse, 0.0f);
			lambda = normalImpulse - vcp->normalImpulse;
			vcp->normalImpulse = normalImpulse;

			Vector3 normalImpulseW = vc->normal * lambda;

			vA -= vc->invMassA * normalImpulseW;
			vB += vc->invMassB * normalImpulseW;
			wA -= vc->invWorldIA * Vector3::cross(rA, normalImpulseW);
			wB += vc->invWorldIB * Vector3::cross(rB, normalImpulseW);

			linearVelocities[vc->indexA] = vA;
			linearVelocities[vc->indexB] = vB;
			angularVelocities[vc->indexA] = wA;
			angularVelocities[vc->indexB] = wB;
		}
	}
}

void ContactSolver::saveImpulse()
{
	for (int i = 0; i < constraintsCount; ++i)
	{
		ContactVelocityConstraint* vc = velocityConstraints + i;
		Contact* contact = contacts[vc->contactIndex];

		for (int j = 0; j < vc->contactPointCount; ++j)
		{
			ContactVelocityConstraintPoint* vcp = vc->contactPoints + j;
			contact->contactPoints[j].normalImpulse = vcp->normalImpulse;
			contact->contactPoints[j].tangentImpulse = vcp->tangentImpulse;
		}
	}
}

bool ContactSolver::solvePositionConstraints()
{
	float32 minSeparation = 0.0f;

	for (int i = 0; i < constraintsCount; ++i)
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