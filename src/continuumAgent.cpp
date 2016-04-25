//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#include "SteerLib.h"
#include "continuumAgent.h"
#include "continuumModule.h"

/// @file ContinuumAgent.cpp
/// @brief Implements the ContinuumAgent class.

#define MAX_FORCE_MAGNITUDE 3.0f
#define MAX_SPEED 1.3f
#define AGENT_MASS 1.0f

#define PRINTS 0

ContinuumAgent::ContinuumAgent(ContinuumGrid *densityVelocityGrid)
{
	Util::Point min = densityVelocityGrid->m_min;
	Util::Point max = densityVelocityGrid->m_max;
	int res_x = densityVelocityGrid->m_res_x;
	int res_z = densityVelocityGrid->m_res_z;

	m_potentialGrid = new PotentialGrid(res_x, res_z, min, max, densityVelocityGrid);
	_enabled = false;
}

ContinuumAgent::~ContinuumAgent()
{
	delete m_potentialGrid;
	if (_enabled) {
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		gSpatialDatabase->removeObject( this, bounds);
	}
}

void ContinuumAgent::disable()
{
	Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
	gSpatialDatabase->removeObject( this, bounds);
	_enabled = false;
}

void ContinuumAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = initialConditions.direction;
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * Util::normalize(initialConditions.direction);

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		gSpatialDatabase->addObject( this, newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		gSpatialDatabase->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0) {
		throw Util::GenericException("No goals were specified!\n");
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
			_goalQueue.push(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom) {
				// if the goal is random, we must randomly generate the goal.
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; ContinuumAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
	}

	// agent starts off traveling towards goal at max speed
	if (!_goalQueue.empty()) {
		Util::Vector vectorToGoal = _goalQueue.front().targetLocation - _position;
		_velocity = (vectorToGoal / vectorToGoal.norm()) * MAX_SPEED;
	}

	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}


void ContinuumAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	//std::cout << "updating agent" << std::endl;
	//std::cout << "agent is at " << _position.x << " " << _position.z;
	int x;
	int z;
	m_potentialGrid->m_potential->getIndicesForCoordinate(_position.x, _position.z, x, z);
	//std::cout << " which is cell " << x << " " << z << std::endl;

	// for this function, we assume that all goals are of type GOAL_TYPE_SEEK_STATIC_TARGET.
	// the error check for this was performed in reset().
	Util::AutomaticFunctionProfiler profileThisFunction( &ContinuumGlobals::gPhaseProfilers->aiProfiler );

	Util::Vector vectorToGoal = _goalQueue.front().targetLocation - _position;

	// it is up to the agent to decide what it means to have "accomplished" or "completed" a goal.
	// for the simple AI, if the agent's distance to its goal is less than its radius, then the agent has reached the goal.
	if (vectorToGoal.lengthSquared() < _radius * _radius) {
		_goalQueue.pop();
		if (_goalQueue.size() != 0) {
			// in this case, there are still more goals, so start steering to the next goal.
			vectorToGoal = _goalQueue.front().targetLocation - _position;
		}
		else {
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			return;
		}
	}

	// use a new speed vector from the potential grid as a force in this timestep.
	// the euler integration step will clamp this vector to a reasonable value, if needed.
	// also, the Euler step updates the agent's position in the spatial database.
	Util::Point goal_pos = _goalQueue.front().targetLocation;
	//std::cout << "updating grid with target " << pos.x << " " << pos.y << std::endl;

	m_potentialGrid->update(goal_pos);
	//m_potentialGrid->m_potential->printGrid(x, z);
	//std::cout << "done updating grid with target " << pos.x << " " << pos.y << std::endl;
	// renormalize gradient
	float dpW = m_potentialGrid->m_d_potential_W->getByCoordinate(_position.x, _position.z);
	float dpN = m_potentialGrid->m_d_potential_N->getByCoordinate(_position.x, _position.z);
	float dpS = m_potentialGrid->m_d_potential_S->getByCoordinate(_position.x, _position.z);
	float dpE = m_potentialGrid->m_d_potential_E->getByCoordinate(_position.x, _position.z);

	//float speedW = m_potentialGrid->m_speed_W->getByCoordinate(_position.x, _position.z);
	//float speedN = m_potentialGrid->m_speed_N->getByCoordinate(_position.x, _position.z);
	//float speedS = m_potentialGrid->m_speed_S->getByCoordinate(_position.x, _position.z);
	//float speedE = m_potentialGrid->m_speed_E->getByCoordinate(_position.x, _position.z);

	Util::Vector p_grad = Util::Vector(dpE - dpW, 0.0f, dpN - dpS);
	// multiply by neg speed in direction, since traveling opposite gradient
	if (p_grad.x > 0.0f) {
		// going east
		p_grad.x *= -MAX_SPEED;
	}
	else {
		p_grad.x *= -MAX_SPEED;
	}

	if (p_grad.z > 0.0f) {
		// going north
		p_grad.z *= -MAX_SPEED;
	}
	else {
		p_grad.z *= -MAX_SPEED;
	}
	//std::cout << "positi is " << _position.x << " " << _position.z << std::endl;
	//std::cout << "target is " << goal_pos.x << " " << goal_pos.y << std::endl;
	//std::cout << "speeds are " << speedW << " " << speedN << " " << speedS << " " << speedE << std::endl;
	//std::cout << "potens are " << dpW << " " << dpN << " " << dpS << " " << dpE << std::endl;
	//std::cout << "grad is " << p_grad.x << " " << p_grad.z << std::endl;

	_doEulerStep(p_grad, dt);

}

SteerLib::EngineInterface * ContinuumAgent::getSimulationEngine()
{
	return gEngine;
}

void ContinuumAgent::draw()
{
#ifdef ENABLE_GUI
	// if the agent is selected, do some annotations just for demonstration
	if (gEngine->isAgentSelected(this)) {
		Util::Ray ray;
		ray.initWithUnitInterval(_position, _forward);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
		if (gSpatialDatabase->trace(ray, t, objectFound, this, false)) {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
		}
	}
	else {
		Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gGray40);
	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}
#endif
}


void ContinuumAgent::_doEulerStep(const Util::Vector & steeringDecisionForce, float dt)
{
	
	// compute acceleration, _velocity, and newPosition by a simple Euler step
	/*
	const Util::Vector clippedForce = Util::clamp(steeringDecisionForce, MAX_FORCE_MAGNITUDE);
	Util::Vector acceleration = (clippedForce / AGENT_MASS);
	_velocity = _velocity + (dt*acceleration);
	_velocity = clamp(_velocity, MAX_SPEED);  // clamp _velocity to the max speed
	*/
	_velocity = steeringDecisionForce;
	_velocity = clamp(_velocity, MAX_SPEED);  // clamp _velocity to the max speed
	const Util::Point newPosition = _position + (dt*_velocity);

	/*
	// For this simple agent, we just make the orientation point along the agent's current velocity.
	if (_velocity.lengthSquared() != 0.0f) {
		_forward = normalize(_velocity);
	} */

	// update the database with the new agent's setup
	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	Util::AxisAlignedBox newBounds(newPosition.x - _radius, newPosition.x + _radius, 0.0f, 0.0f, newPosition.z - _radius, newPosition.z + _radius);
	gSpatialDatabase->updateObject( this, oldBounds, newBounds);

	_position = newPosition;
}
