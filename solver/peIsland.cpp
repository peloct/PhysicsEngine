#include"peIsland.h"
#include"peContactSolver.h"
#include"../memory/peStackAllocator.h"
#include"../peRigidbody.h"
#include"../common/peProfile.h"
#include"../common/peTimer.h"

Island::Island(int32 bodyCapacity, int32 contactCapacity, StackAllocator* stackAllocator)
{
	this->stackAllocator = stackAllocator;

	rigidbodyCount = 0;
	contactCount = 0;

	rigidbodies = (Rigidbody**)stackAllocator->allocate(bodyCapacity * sizeof(Rigidbody*));
	contacts = (Contact**)stackAllocator->allocate(contactCapacity * sizeof(Contact*));
	positions = (Vector3*)stackAllocator->allocate(bodyCapacity * sizeof(Vector3));
	orientations = (Quaternion*)stackAllocator->allocate(bodyCapacity * sizeof(Quaternion));
	linearVelocities = (Vector3*)stackAllocator->allocate(bodyCapacity * sizeof(Vector3));
	angularVelocities = (Vector3*)stackAllocator->allocate(bodyCapacity * sizeof(Vector3));
}

Island::~Island()
{
	stackAllocator->free(angularVelocities);
	stackAllocator->free(linearVelocities);
	stackAllocator->free(orientations);
	stackAllocator->free(positions);
	stackAllocator->free(contacts);
	stackAllocator->free(rigidbodies);
}

void Island::solve(const TimeStep& timeStep, const Vector3& gravity, Profile* profile)
{
	Timer timer;

	float32 dt = timeStep.dt;

	for (int i = 0; i < rigidbodyCount; ++i)
	{
		Rigidbody* each = rigidbodies[i];

		Vector3 v = each->linearVelocity;
		Vector3 w = each->angularVelocity;

		if (each->bodyType == RigidbodyType::dynamicBody)
		{
			const float32 invMass = each->invMass;
			const Matrix3x3& invI = each->invWorldInertiaTensor;

			v = v + (gravity + invMass * each->force) * dt;
			w = w + invI * each->torque * dt;

			v *= 1.0f / (1.0f + dt * each->linearDamping);
			w *= 1.0f / (1.0f + dt * each->angularDamping);
		}

		positions[i] = each->massPosition;
		orientations[i] = each->orientation;
		linearVelocities[i] = v;
		angularVelocities[i] = w;
	}

	ContactSolverDef contactSolverDef;
	contactSolverDef.contactCount = contactCount;
	contactSolverDef.contactsInIsland = contacts;
	contactSolverDef.stackAllocator = stackAllocator;
	contactSolverDef.positions = positions;
	contactSolverDef.orientations = orientations;
	contactSolverDef.linearVelocities = linearVelocities;
	contactSolverDef.angularVelocities = angularVelocities;

	ContactSolver contactSolver(contactSolverDef);

	contactSolver.initVelocityConstraints();

	contactSolver.warmStart();

	timer.reset();
	for (int i = 0; i < timeStep.velConstraintIterCnt; ++i)
		contactSolver.solveVelocityConstraints();
	profile->solvingVC += timer.getMilliseconds();

	contactSolver.saveImpulse();

	for (int i = 0; i < rigidbodyCount; ++i)
	{
		Vector3 p = positions[i];
		Quaternion q = orientations[i];
		Vector3 v = linearVelocities[i];
		Vector3 w = angularVelocities[i];

		// 변형의 최대 바운더리를 줄 것.

		q.addScaledVector(w, timeStep.dt);
		q.normailize();

		positions[i] = p + v * timeStep.dt;
		orientations[i] = q;
	}

	timer.reset();
	bool positionSolved = false;
	for (int i = 0; i < timeStep.posConstraintIterCnt; ++i)
		if (contactSolver.solvePositionConstraints())
		{
			positionSolved = true;
			break;
		}
	profile->solvingPC += timer.getMilliseconds();

	for (int i = 0; i < rigidbodyCount; ++i)
	{
		Rigidbody* each = rigidbodies[i];

		Vector3 p = positions[i];
		Quaternion q = orientations[i];
		Vector3 v = linearVelocities[i];
		Vector3 w = angularVelocities[i];

		each->massPosition = p;
		each->orientation = q;
		each->linearVelocity = v;
		each->angularVelocity = w;
		each->updateTransformDependants();
		each->synchronizeTransform();
	}
}