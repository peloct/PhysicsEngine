#pragma once

#include"peJoint.h"

struct SpringJointDef : public JointDef
{
	SpringJointDef() :  JointDef(JointType::eSpringJoint), bodyA(nullptr), bodyB(nullptr), localAnchorA(), localAnchorB(), spring(0.0f) {}

	Rigidbody* bodyA;
	Rigidbody* bodyB;
	Vector3 localAnchorA;
	Vector3 localAnchorB;
	float32 spring;
};

class SpringJoint : public Joint
{
public:
	SpringJoint(const SpringJointDef& def);
	
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

	float32 spring;
	float32 referenceDistance;
};