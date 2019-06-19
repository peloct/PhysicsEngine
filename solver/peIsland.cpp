#include"peIsland.h"
#include"peSISolver.h"
#include"peNNCGSolver.h"
#include"../memory/peStackAllocator.h"
#include"../peRigidbody.h"
#include"../common/peProfile.h"
#include"../common/peTimer.h"

// 0.01 degree per sec
#define SLEEP_ANGULARVEL_TOL 2.0f / 180.0f * 3.141592f
#define SLEEP_TIME 2.0f

const float32 sleepLinearVelTolSqr = 0.01f * 0.01f;
const float32 sleepAngularVelTolSqr = SLEEP_ANGULARVEL_TOL * SLEEP_ANGULARVEL_TOL;

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

IslandInfo Island::solve(const TimeStep& timeStep, const Vector3& gravity, const IslandInfo& islandInfo, Profile* profile)
{
	IslandInfo ret;
	ret.rigidbodyCount = rigidbodyCount;

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

	NNCGSolverDef solverDef;
	solverDef.timeStep = timeStep;
	solverDef.rigidbodyCount = rigidbodyCount;
	solverDef.contactCount = contactCount;
	solverDef.contactsInIsland = contacts;
	solverDef.stackAllocator = stackAllocator;
	solverDef.positions = positions;
	solverDef.orientations = orientations;
	solverDef.linearVelocities = linearVelocities;
	solverDef.angularVelocities = angularVelocities;
	solverDef.prevStepInfo = islandInfo.isValid ? islandInfo.nncgSolverPrevStepInfo : NNCGSolverStepInfo();

	NNCGSolver solver(solverDef);

	solver.initVelocityConstraints();

	solver.warmStart();

	timer.reset();

	for (int i = 0; i < timeStep.velocityIteration; ++i)
	{
		solver.solveVelocityConstraints();
		if (solver.hit)
			++profile->hitCount;
	}

	profile->solvingVC += timer.getMilliseconds();
	ret.nncgSolverPrevStepInfo = solver.getStepInfo();
	solver.saveImpulse();

	solver.applyeDelta();

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
	for (int i = 0; i < timeStep.positionIteration; ++i)
		if (solver.solvePositionConstraints())
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

	float32 minSleepTime = 100000.0f;
	for (int i = 0; i < rigidbodyCount; ++i)
	{
		Rigidbody* each = rigidbodies[i];

		if (each->bodyType == RigidbodyType::staticBody)
			continue;

		if (each->linearVelocity.sqrMagnitude() > sleepLinearVelTolSqr || each->angularVelocity.sqrMagnitude() > sleepAngularVelTolSqr)
		{
			each->sleepTimer = 0.0f;
			minSleepTime = 0.0f;
		}
		else
		{
			each->sleepTimer += dt;
			minSleepTime = peMinf(minSleepTime, each->sleepTimer);
		}
	}

	if (minSleepTime > SLEEP_TIME && positionSolved)
	{
		for (int i = 0; i < rigidbodyCount; ++i)
		{
			Rigidbody* each = rigidbodies[i];

			if (each->bodyType == RigidbodyType::staticBody)
				continue;

			each->setAwake(false);
		}
	}

	return ret;
}