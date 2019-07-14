#include"peNNCGSolver.h"
#include"../memory/peStackAllocator.h"
#include"../collision/peContact.h"
#include"../peFixture.h"
#include"../peRigidbody.h"
#include"../joint/peJoint.h"

struct NNCGSolver::ContactVCCacheRef
{
	ContactPoint* cp;
	int32 dirIndex;
};

NNCGSolver::NNCGSolver(const NNCGSolverData& data)
{
#ifdef DEBUG_LOG
	peLog("   === NNCGSolver Init ===\n");
#endif

	stackAllocator = data.stackAllocator;

	int32 contactCount = data.contactCount;
	contacts = data.contactsInIsland;

	jointCount = data.jointCount;
	joints = data.jointsInIsland;

	positions = data.positions;
	orientations = data.orientations;
	linearVelocities = data.linearVelocities;
	angularVelocities = data.angularVelocities;

	jointVCCount = 0;
	for (int i = 0; i < jointCount; ++i)
		jointVCCount += joints[i]->getVelConstraintCount();

	contactVCCount = 0;
	for (int i = 0; i < contactCount; ++i)
		contactVCCount += contacts[i]->contactPointCount;
	contactVCCount *= 3;

	velConstraintsCount = jointVCCount + contactVCCount;
	contactVCCacheRefs = (ContactVCCacheRef*)stackAllocator->allocate(contactVCCount * sizeof(ContactVCCacheRef));
	velocityConstraints = (VelocityConstraint*)stackAllocator->allocate(velConstraintsCount * sizeof(VelocityConstraint));

	contactPCCount = contactCount;
	positionConstraints = (ContactPositionConstraint*)stackAllocator->allocate(contactPCCount * sizeof(ContactPositionConstraint));

	rigidbodyCount = data.rigidbodyCount;
	dLinearV = (Vector3*)stackAllocator->allocate(rigidbodyCount * sizeof(Vector3));
	dAngularV = (Vector3*)stackAllocator->allocate(rigidbodyCount * sizeof(Vector3));

	prevGradientMagSqr = data.prevStepInfo.prevGradientMagSqr;
	curGradientMagSqr = data.prevStepInfo.curGradientMagSqr;

	for (int i = 0; i < rigidbodyCount; ++i)
	{
		dLinearV[i].setZero();
		dAngularV[i].setZero();
	}

	int vcIndex = 0;

	for (int i = 0; i < jointCount; ++i)
	{
		Joint* joint = joints[i];
		joint->createVelConstraints(data, velocityConstraints + vcIndex, dLinearV, dAngularV);
		vcIndex += joint->getVelConstraintCount();
	}

	int32 contactCachRefID = 0;
	for (int i = 0; i < contactCount; ++i)
	{
		Contact* contact = contacts[i];

		Fixture* fixtureA;
		Fixture* fixtureB;

		if (contact->contactFaceOwner == ContactFaceOwner::fixtureA)
		{
			fixtureA = contact->fixtureA;
			fixtureB = contact->fixtureB;
		}
		else
		{
			fixtureA = contact->fixtureB;
			fixtureB = contact->fixtureA;
		}

		const Rigidbody* rigidbodyA = fixtureA->getRigidbody();
		const Rigidbody* rigidbodyB = fixtureB->getRigidbody();

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

		Matrix4x4 bodyLocalToWorldA = fixtureA->getBodyLocalToWorld();
		Matrix4x4 bodyLocalToWorldB = fixtureB->getBodyLocalToWorld();

		Vector3 dir[3];
		dir[0] = bodyLocalToWorldA.transformDirection(contact->localNormal);
		dir[1] = bodyLocalToWorldA.transformDirection(contact->localTangent1);
		dir[2] = bodyLocalToWorldA.transformDirection(contact->localTangent2);

#ifdef DEBUG_LOG 	
		peLog("   (%d)th contact [%d]\n", i, contact->guid);
		peLog("   bodyA : %d, bodyB : %d\n", rigidbodyA->guid, rigidbodyB->guid);
		peLog("   bodyA index : %d, bodyB index : %d\n", indexA, indexB);
		peLog("   pA : (%f, %f, %f), pB : (%f, %f, %f)\n", pA.x, pA.y, pA.z, pB.x, pB.y, pB.z);
		peLog("   vA : (%f, %f, %f), vB : (%f, %f, %f)\n", vA.x, vA.y, vA.z, vB.x, vB.y, vB.z);
		peLog("   wA : (%f, %f, %f), wB : (%f, %f, %f)\n", wA.x, wA.y, wA.z, wB.x, wB.y, wB.z);
		peLog("   n : (%f, %f, %f), t1 : (%f, %f, %f), t2 : (%f, %f, %f)\n", dir[0].x, dir[0].y, dir[0].z, dir[1].x, dir[1].y, dir[1].z, dir[2].x, dir[2].y, dir[2].z);
#endif

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
			cachedImpulse[0] = cp->normalImpulse * data.timeStep.ratio;
			cachedImpulse[1] = cp->tangentImpulse.x * data.timeStep.ratio;
			cachedImpulse[2] = cp->tangentImpulse.y * data.timeStep.ratio;
			float32 fGradient[3] = { cp->fGradient.x, cp->fGradient.y, cp->fGradient.z };
			float32 direction[3] = { cp->direction.x, cp->direction.y, cp->direction.z };

			Vector3 worldPoint = bodyLocalToWorldB.transformPoint(cp->localPoint);

			Vector3 rA = worldPoint - pA;
			Vector3 rB = worldPoint - pB;

#ifdef DEBUG_LOG
			peLog("   contactPoint [%d]\n", j);
			peLog("      cached impulse n : (%f), t1 : (%f), t2 : (%f)\n", cachedImpulse[0], cachedImpulse[1], cachedImpulse[2]);
			peLog("      worldPoint : (%f, %f, %f)\n", worldPoint.x, worldPoint.y, worldPoint.z);
#endif // DEBUG_LOG

			for (int k = 0; k < 3; ++k)
			{
				VelocityConstraint* vc = velocityConstraints + vcIndex;
				vc->doSolve = true;

				ContactVCCacheRef& cacheRef = contactVCCacheRefs[contactCachRefID++];
				vc->cacheRef = &cacheRef;
				cacheRef.cp = cp;
				cacheRef.dirIndex = k;

				Vector3 dirA = -dir[k];
				Vector3 dirB = dir[k];

				Jsp& jsp = vc->j;

				jsp.linearA = dirA;
				jsp.angularA = Vector3::cross(rA, dirA);
				jsp.linearB = dirB;
				jsp.angularB = Vector3::cross(rB, dirB);
				
				Bsp& bsp = vc->b;

				bsp.linearA = invMassA * jsp.linearA;
				bsp.angularA = invWorldIA * jsp.angularA;
				bsp.linearB = invMassB * jsp.linearB;
				bsp.angularB = invWorldIB * jsp.angularB;

				float32 impulse = cachedImpulse[k];

				vc->impulse = impulse;

				dLinearV[indexA] += impulse * bsp.linearA;
				dAngularV[indexA] += impulse * bsp.angularA;
				dLinearV[indexB] += impulse * bsp.linearB;
				dAngularV[indexB] += impulse * bsp.angularB;

				vc->d =
					Vector3::dot(jsp.linearA, bsp.linearA)
					+ Vector3::dot(jsp.angularA, bsp.angularA)
					+ Vector3::dot(jsp.linearB, bsp.linearB)
					+ Vector3::dot(jsp.angularB, bsp.angularB);


				vc->indexA = indexA;
				vc->indexB = indexB;

				Vector3 relVel = vB + Vector3::cross(wB, rB) - vA - Vector3::cross(wA, rA);
				float32 relVelToDir = Vector3::dot(relVel, dirB);

				vc->dependentImpulse = NULL_ID;
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

#ifdef DEBUG_LOG
				peLog("      vel constraint [%d]\n", vcIndex);
				peLog("         dir index [%d]\n", k);
				peLog("         dirB : (%f, %f, %f)\n", dirB.x, dirB.y, dirB.z);
				peLog("         jsp.linearA : (%f, %f, %f)\n", jsp.linearA.x, jsp.linearA.y, jsp.linearA.z);
				peLog("         jsp.angularA : (%f, %f, %f)\n", jsp.angularA.x, jsp.angularA.y, jsp.angularA.z);
				peLog("         jsp.linearB : (%f, %f, %f)\n", jsp.linearB.x, jsp.linearB.y, jsp.linearB.z);
				peLog("         jsp.angularB : (%f, %f, %f)\n", jsp.angularB.x, jsp.angularB.y, jsp.angularB.z);
				peLog("         bsp.linearA : (%f, %f, %f)\n", bsp.linearA.x, bsp.linearA.y, bsp.linearA.z);
				peLog("         bsp.angularA : (%f, %f, %f)\n", bsp.angularA.x, bsp.angularA.y, bsp.angularA.z);
				peLog("         bsp.linearB : (%f, %f, %f)\n", bsp.linearB.x, bsp.linearB.y, bsp.linearB.z);
				peLog("         bsp.angularB : (%f, %f, %f)\n", bsp.angularB.x, bsp.angularB.y, bsp.angularB.z);
				peLog("         d : %f\n", vc->d);
				peLog("         desiredDelV : %f\n", vc->desiredDelV);
				peLog("         dLinearV[indexA] : (%f, %f, %f)\n", dLinearV[indexA].x, dLinearV[indexA].y, dLinearV[indexA].z);
				peLog("         dAngularV[indexA] : (%f, %f, %f)\n", dAngularV[indexA].x, dAngularV[indexA].y, dAngularV[indexA].z);
				peLog("         dLinearV[indexB] : (%f, %f, %f)\n", dLinearV[indexB].x, dLinearV[indexB].y, dLinearV[indexB].z);
				peLog("         dAngularV[indexB] : (%f, %f, %f)\n", dAngularV[indexB].x, dAngularV[indexB].y, dAngularV[indexB].z);
#endif

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
	stackAllocator->free(contactVCCacheRefs);
}

void NNCGSolver::PGS()
{
#ifdef DEBUG_LOG
	peLog("   === NNCGSolver PGS ===\n");
#endif // DEBUG_LOG

	for (int i = 0; i < velConstraintsCount; ++i)
	{
		VelocityConstraint* vc = velocityConstraints + i;

		if (!vc->doSolve)
			continue;

		int32 indexA = vc->indexA;
		int32 indexB = vc->indexB;

		const Bsp& b = vc->b;

		float32 delV = Vector3::dot(vc->j.linearA, dLinearV[indexA]) + Vector3::dot(vc->j.angularA, dAngularV[indexA]);
		if (indexB != NULL_ID)
			delV += Vector3::dot(vc->j.linearB, dLinearV[indexB]) + Vector3::dot(vc->j.angularB, dAngularV[indexB]);

		float32 dImpulse = (vc->desiredDelV - delV) / vc->d;

		float32 impulse0 = vc->impulse;

		float32 minImpulse = vc->minImpulse;
		float32 maxImpulse = vc->maxImpulse;

		if (vc->dependentImpulse != NULL_ID)
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

		if (indexB != NULL_ID)
		{
			dLinearV[indexB] += dImpulse * b.linearB;
			dAngularV[indexB] += dImpulse * b.angularB;
		}

#ifdef DEBUG_LOG
		peLog("   vel constraint [%d]\n", i);
		peLog("      indexA : %d, indexB : %d\n", indexA, indexB);
		peLog("      delV : %f\n", delV);
		peLog("      dImpulse : %f\n", dImpulse);
		peLog("      impulse : %f\n", impulse);
		peLog("      solved delV : %f\n", delV);
		peLog("      dLinearV[indexA] : (%f, %f, %f)\n", dLinearV[indexA].x, dLinearV[indexA].y, dLinearV[indexA].z);
		peLog("      dAngularV[indexA] : (%f, %f, %f)\n", dAngularV[indexA].x, dAngularV[indexA].y, dAngularV[indexA].z);
		peLog("      dLinearV[indexB] : (%f, %f, %f)\n", dLinearV[indexB].x, dLinearV[indexB].y, dLinearV[indexB].z);
		peLog("      dAngularV[indexB] : (%f, %f, %f)\n", dAngularV[indexB].x, dAngularV[indexB].y, dAngularV[indexB].z);
#endif // DEBUG_LOG
	}
}

void NNCGSolver::solveVelocityConstraints()
{
	if (prevGradientMagSqr > 0.0f)
	{
		float32 beta = curGradientMagSqr / prevGradientMagSqr;
	
		if (beta > 1)
		{
			for (int i = 0; i < velConstraintsCount; ++i)
			{
				VelocityConstraint* vc = velocityConstraints + i;
				vc->searchDir = 0;
			}
		}
		else
		{
			for (int i = 0; i < velConstraintsCount; ++i)
			{
				VelocityConstraint* vc = velocityConstraints + i;
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
			VelocityConstraint* vc = velocityConstraints + i;
			vc->searchDir = -vc->fGradient;
		}
	}
	
	prevGradientMagSqr = curGradientMagSqr;
	curGradientMagSqr = 0.0f;

	PGS();
}

void NNCGSolver::saveImpulse()
{
	VelocityConstraint* jointVCs = velocityConstraints;
	VelocityConstraint* contactVCs = velocityConstraints + jointVCCount;

	for (int i = 0; i < jointVCCount;)
	{
		VelocityConstraint* vc = jointVCs + i;
		Joint* joint = (Joint*)vc->cacheRef;
		joint->saveImpulse(vc);
		i += joint->getVelConstraintCount();
	}

	for (int i = 0; i < contactVCCount; ++i)
	{
		VelocityConstraint* vc = contactVCs + i;
		ContactVCCacheRef* cacheRef = (ContactVCCacheRef*)vc->cacheRef;
		ContactPoint* cp = cacheRef->cp;
		
		if (cacheRef->dirIndex == 0)
		{
			cp->normalImpulse = vc->impulse;
			cp->direction.x = vc->searchDir;
			cp->fGradient.x = vc->fGradient;
		}
		else if (cacheRef->dirIndex == 1)
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

	for (int i = 0; i < contactPCCount; ++i)
	{
		ContactPositionConstraint* pc = positionConstraints + i;

		int32 refIndex = pc->indexA;
		int32 incIndex = pc->indexB;
		Vector3 refLocalCenter = pc->localCenterA;
		Vector3 incLocalCenter = pc->localCenterB;
		float32 refInvMass = pc->invMassA;
		float32 incInvMass = pc->invMassB;
		Matrix3x3 refInvI = pc->invIA;
		Matrix3x3 incInvI = pc->invIB;

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

	bool jointOk = true;
	for (int i = 0; i < jointCount; ++i)
	{
		bool isOk = joints[i]->solvePosConstraints(positions, orientations);
		jointOk &= isOk;
	}

	return minSeparation > -1.5 * LINEAR_SLOP && jointOk;
}