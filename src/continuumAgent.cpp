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

ContinuumAgent::ContinuumAgent()
{
	m_potentialGrid = NULL;
	debug = false;
	_enabled = false;
}

void ContinuumAgent::init(ContinuumGrid *densityVelocityGrid) {
	Util::Point min = densityVelocityGrid->m_min;
	Util::Point max = densityVelocityGrid->m_max;
	int res_x = densityVelocityGrid->m_res_x;
	int res_z = densityVelocityGrid->m_res_z;

	m_potentialGrid = new PotentialGrid(res_x, res_z, min, max, densityVelocityGrid);
}

ContinuumAgent::~ContinuumAgent()
{
	if (m_potentialGrid != NULL) delete m_potentialGrid;
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
	m_allGoalsList.clear();
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = initialConditions.direction;
	_radius = initialConditions.radius;
	//_velocity = initialConditions.speed * Util::normalize(initialConditions.direction);

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
			m_allGoalsList.push_back(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom) {
				// if the goal is random, we must randomly generate the goal.
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
				m_allGoalsList.push_back(_goal);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; ContinuumAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
	}

	// agent starts off traveling towards goal at max speed
	//if (!_goalQueue.empty()) {
		//Util::Vector vectorToGoal = _goalQueue.front().targetLocation - _position;
		//_velocity = (vectorToGoal / vectorToGoal.norm()) * MAX_SPEED;
	//}

	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}


void ContinuumAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	if (m_potentialGrid == NULL) return; // don't do a thing until m_potentialGrid is ready


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
	if (vectorToGoal.lengthSquared() < m_potentialGrid->m_potential->m_cell_size_x * m_potentialGrid->m_potential->m_cell_size_z) {
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
	Util::Vector velocity = m_potentialGrid->interpolateVelocity(_position);// MAX_SPEED;
	
	// jitter velocity
	velocity.x += ((float)std::rand() - 0.5f * (float)RAND_MAX) / (float) RAND_MAX;
	velocity.z += ((float)std::rand() - 0.5f * (float)RAND_MAX) / (float)RAND_MAX;

	//velocity = goal_pos - _position;
	_doEulerStep(velocity, dt);

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
		ray.initWithUnitInterval(_position, _velocity);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
		if (gSpatialDatabase->trace(ray, t, objectFound, this, false)) {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
		}

		// draw a debug grid
		//drawSingleGrid(m_potentialGrid->m_speeds_densities->m_density); // draw density splat
		drawSingleGrid(m_potentialGrid->m_potential); // draw potential

		//drawAverageSpeeds(); // draw avg speeds
		
		/* for drawing costs
		drawFaceGrid(m_potentialGrid->m_speeds_densities->m_cost_N,
			m_potentialGrid->m_speeds_densities->m_cost_S,
			m_potentialGrid->m_speeds_densities->m_cost_E,
			m_potentialGrid->m_speeds_densities->m_cost_W); */

		/* for drawing anisotropic speeds
		drawFaceGrid(m_potentialGrid->m_speeds_densities->m_speed_N,
			m_potentialGrid->m_speeds_densities->m_speed_S,
			m_potentialGrid->m_speeds_densities->m_speed_E,
			m_potentialGrid->m_speeds_densities->m_speed_W); */
		
		/* for drawing potential grad
		drawFaceGrid(m_potentialGrid->m_dPotential_N,
			m_potentialGrid->m_dPotential_S,
			m_potentialGrid->m_dPotential_E,
			m_potentialGrid->m_dPotential_W); */
		
		/*
		drawFaceGrid(m_potentialGrid->m_velocity_N,
			m_potentialGrid->m_velocity_S,
			m_potentialGrid->m_velocity_E,
			m_potentialGrid->m_velocity_W);*/

	}
	else {
		Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gGray40);
	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}
#endif
}

void ContinuumAgent::drawSingleGrid(float_grid_2D *gridVals) {
	if (gridVals == NULL) return;

	Util::Point p1 = Util::Point(); // corner
	Util::Point p2 = Util::Point();
	Util::Point p3 = Util::Point();
	Util::Point p4 = Util::Point();

	Color shade(0.0f, 0.0f, 0.0f);
	float maxVal = gridVals->getMaxVal() + 0.001f;
	
	float cell_margin_x = gridVals->m_cell_size_x / 15.0f;
	float cell_margin_z = gridVals->m_cell_size_z / 15.0f;

	float cell_size_wMargin_x = gridVals->m_cell_size_x - 2.0f * cell_margin_x;
	float cell_size_wMargin_z = gridVals->m_cell_size_z - 2.0f * cell_margin_z;

	float temp_val;

	// draw allll the quads
	for (int x = 0; x < gridVals->m_res_x; x++) {
		for (int z = 0; z < gridVals->m_res_z; z++) {
						
			p1 = gridVals->getCornerOfIndex(x, z);

			// create a margin
			p1.x += cell_margin_x;
			p1.z += cell_margin_z;

			p2.x = p1.x + cell_size_wMargin_x;
			p2.z = p1.z;

			p3.x = p2.x;
			p3.z = p2.z + cell_size_wMargin_z;

			p4.x = p3.x - cell_size_wMargin_x;
			p4.z = p3.z;

			temp_val = gridVals->getByIndex(x, z);

			shade.r = (temp_val / maxVal);
			shade.g = (temp_val / maxVal);
			shade.b = (temp_val / maxVal);

			Util::DrawLib::glColor(shade);
			Util::DrawLib::drawQuad(p4, p3, p2, p1);
		}
	}
}

void ContinuumAgent::drawAverageSpeeds() {
	if (m_potentialGrid == NULL) return;

	float_grid_2D *gridVals_x = m_potentialGrid->m_speeds_densities->m_avg_vel_x;
	float_grid_2D *gridVals_z = m_potentialGrid->m_speeds_densities->m_avg_vel_z;

	Util::Point p1 = Util::Point(); // corner
	Util::Point p2 = Util::Point();
	Util::Point p3 = Util::Point();
	Util::Point p4 = Util::Point();

	Color shade(0.0f, 0.0f, 0.0f);
	int ix, iz;
	float maxVal_x = gridVals_x->getMaxVal(ix, iz) + 0.001f;
	float maxVal_z = gridVals_z->getMaxVal(ix, iz) + 0.001f;

	float cell_margin_x = gridVals_x->m_cell_size_x / 15.0f;
	float cell_margin_z = gridVals_z->m_cell_size_z / 15.0f;

	float cell_size_wMargin_x = gridVals_x->m_cell_size_x - 2.0f * cell_margin_x;
	float cell_size_wMargin_z = gridVals_z->m_cell_size_z - 2.0f * cell_margin_z;

	float temp_val_x;
	float temp_val_z;

	// draw allll the quads
	for (int x = 0; x < gridVals_x->m_res_x; x++) {
		for (int z = 0; z < gridVals_x->m_res_z; z++) {

			p1 = gridVals_x->getCornerOfIndex(x, z);

			// create a margin
			p1.x += cell_margin_x;
			p1.z += cell_margin_z;

			p2.x = p1.x + cell_size_wMargin_x;
			p2.z = p1.z;

			p3.x = p2.x;
			p3.z = p2.z + cell_size_wMargin_z;

			p4.x = p3.x - cell_size_wMargin_x;
			p4.z = p3.z;

			temp_val_x = abs(gridVals_x->getByIndex(x, z));
			temp_val_z = abs(gridVals_z->getByIndex(x, z));

			shade.r = (temp_val_x / maxVal_x);
			shade.g = 0.0f;
			shade.b = (temp_val_z / maxVal_z);

			Util::DrawLib::glColor(shade);
			Util::DrawLib::drawQuad(p4, p3, p2, p1);
		}
	}
}

void ContinuumAgent::drawFaceGrid(float_grid_2D *gridVals_N, float_grid_2D *gridVals_S, float_grid_2D *gridVals_E, float_grid_2D *gridVals_W) {
	if (m_potentialGrid == NULL) return;

	if (gridVals_N == NULL ||
		gridVals_S == NULL ||
		gridVals_E == NULL ||
		gridVals_W == NULL) return;

	Util::Point p_N = Util::Point();
	Util::Point p_S = Util::Point();
	Util::Point p_E = Util::Point();
	Util::Point p_W = Util::Point();
	Util::Point center;

	float f_N, f_S, f_E, f_W;

	Color shade(0.0f, 0.0f, 0.0f);
	int ix, iz;
	float maxVal_N = gridVals_N->getMaxValAbs() + 0.001f;
	float maxVal_S = gridVals_S->getMaxValAbs() + 0.001f;
	float maxVal_E = gridVals_E->getMaxValAbs() + 0.001f;
	float maxVal_W = gridVals_W->getMaxValAbs() + 0.001f;

	float halfCell_x = gridVals_N->m_cell_size_x / 2.0f;
	float halfCell_z = gridVals_N->m_cell_size_z / 2.0f;

	// draw allll the thingies
	for (int x = 0; x < gridVals_N->m_res_x; x++) {
		for (int z = 0; z < gridVals_N->m_res_z; z++) {

			center = gridVals_N->getCornerOfIndex(x, z);
			center.x += halfCell_x;
			center.z += halfCell_z;

			// draw north
			p_N = center;
			p_N.z += halfCell_z * 0.5f;
			f_N = gridVals_N->getByIndex(x, z);
			shade = Color(0.0f, 0.0f, 0.0f);
			if (f_N < 0.0f) {
				shade.b = fabs(f_N) / maxVal_N;
			}
			else {
				shade.r = fabs(f_N) / maxVal_N;
			}
			Util::DrawLib::drawCircle(p_N, shade, halfCell_z / 4.0f);

			// draw south
			p_S = center;
			p_S.z -= halfCell_z * 0.5f;
			f_S = gridVals_S->getByIndex(x, z);
			shade = Color(0.0f, 0.0f, 0.0f);
			if (f_N < 0.0f) {
				shade.b = fabs(f_S) / maxVal_S;
			}
			else {
				shade.r = fabs(f_S) / maxVal_S;
			}
			Util::DrawLib::drawCircle(p_S, shade, halfCell_z / 4.0f);

			// draw East
			p_E = center;
			p_E.x += halfCell_x * 0.5f;
			f_E = gridVals_E->getByIndex(x, z);
			shade = Color(0.0f, 0.0f, 0.0f);
			if (f_E < 0.0f) {
				shade.b = fabs(f_E) / maxVal_E;
			}
			else {
				shade.r = fabs(f_E) / maxVal_E;
			}
			Util::DrawLib::drawCircle(p_E, shade, halfCell_x / 4.0f);

			// draw West
			p_W = center;
			p_W.x -= halfCell_x * 0.5f;
			f_W = gridVals_W->getByIndex(x, z);
			shade = Color(0.0f, 0.0f, 0.0f);
			if (f_W < 0.0f) {
				shade.b = fabs(f_W) / maxVal_W;
			}
			else {
				shade.r = fabs(f_W) / maxVal_W;
			}
			Util::DrawLib::drawCircle(p_W, shade, halfCell_x / 4.0f);

		}
	}
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
