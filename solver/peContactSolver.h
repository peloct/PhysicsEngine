#pragma once

#include"../collision/peContact.h"
#include"../common/peTimeStep.h"

class ContactSolver
{
public:
	virtual void initVelocityConstraints() {};
	virtual void warmStart() {};
	virtual void solveVelocityConstraints() = 0;
	virtual void saveImpulse() {};
	virtual void applyeDelta() {};
	virtual bool solvePositionConstraints() = 0;
};