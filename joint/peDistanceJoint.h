#pragma once

#include"peJoint.h"

struct DistanceJointDef : public JointDef
{
	DistanceJointDef() :  JointDef(JointType::eDistanceJoint), bodyA(nullptr), bodyB(nullptr), localAnchorA(), localAnchorB(),
		targetDistance(-1.0f), spring(0.0f), damping(0.0f), isMin(false), isMax(false), restitution(0.0f) {}

	Rigidbody* bodyA;
	Rigidbody* bodyB;
	Vector3 localAnchorA;
	Vector3 localAnchorB;

	float32 targetDistance;
	float32 spring;
	float32 damping;
	bool isMin;
	bool isMax;
	float32 restitution;
};

class DistanceJoint : public Joint
{
public:
	DistanceJoint(const DistanceJointDef& def);
	
protected:
	int32 getVelConstraintCount() const override;
	void createVelConstraints(const NNCGSolverData& solverData, VelocityConstraint* velocityConstraints, Vector3* dLinearV, Vector3* dAngularV) override;
	void saveImpulse(VelocityConstraint* velocityConstraints) override;
	bool solvePosConstraints(Vector3* positions, Quaternion* orientations) const override;

private:
	Vector3 localAnchorA;
	Vector3 localAnchorB;
	Vector3 centerPosA;
	Vector3 centerPosB;

	float32 fGradient;
	float32 searchDir;
	float32 impulse;

	float32 targetDistance;
	float32 damping;
	float32 spring;
	bool isMin;
	bool isMax;
	float32 restitution;
};