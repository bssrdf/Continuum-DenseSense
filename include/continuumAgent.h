//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#ifndef __CONTINUUM_AGENT__
#define __CONTINUUM_AGENT__

/// @file continuumAgent.h
/// @brief Declares the ContinuumAgent class.

#include <queue>
#include "SteerLib.h"
#include "continuumModule.h"
#include "continuumAgent.h"
#include "continuumGrid.h"
#include "potentialGrid.hpp"
#include "floatGrid2D.h"

/**
 * @brief An example agent with very basic AI, that is part of the simpleAI plugin.
 *
 * This agent performs extremely simple AI using forces and Euler integration, simply
 * steering towards static goals without avoiding any other agents or static obstacles.
 * Agents that are "selected" in the GUI will have some simple annotations that
 * show how the spatial database and engine interface can be used.
 *
 * This class is instantiated when the engine calls SimpleAIModule::createAgent().
 *
 */

class ContinuumModule; // forward declaration

class ContinuumAgent : public SteerLib::AgentInterface
{
public:
	ContinuumAgent();
	void init(PotentialGrid *potentialGrid);
	~ContinuumAgent();
	void reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo);
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void disable();
	void draw();
	
	// for debugging
	void drawSingleGrid(float_grid_2D *gridVals);
	void drawAverageSpeeds();
	void drawFaceGrid(float_grid_2D *gridVals_N, float_grid_2D *gridVals_S, float_grid_2D *gridVals_E, float_grid_2D *gridVals_W);

	bool enabled() const { return _enabled; }
	Util::Point position() const { return _position; }
	Util::Vector forward() const { return _forward; }
	Util::Vector velocity() const { return _velocity; }
	float radius() const { return _radius; }
	const SteerLib::AgentGoalInfo & currentGoal() const { return _goalQueue.front(); }
	size_t id() const { return 0;}
	const std::queue<SteerLib::AgentGoalInfo> & agentGoals() const { throw Util::GenericException("agentGoals() not implemented yet"); }
	void addGoal(const SteerLib::AgentGoalInfo & newGoal) { throw Util::GenericException("addGoals() not implemented yet for ContinuumAgent"); }
	void clearGoals() { throw Util::GenericException("clearGoals() is not implemented yet for ContinuumAgent"); }

	void insertAgentNeighbor(const SteerLib::AgentInterface *agent, float &rangeSq) { throw Util::GenericException("insertAgentNeighbor not implemented yet for BenchmarkAgent"); }
	void setParameters(SteerLib::Behaviour behave)
	{
		throw Util::GenericException("setParameters() not implemented yet for this Agent");
	}
	/// @name The SteerLib::SpatialDatabaseItemInterface
	/// @brief These functions are required so that the agent can be used by the SteerLib::GridDatabase2D spatial database;
	/// The Util namespace helper functions do the job nicely for basic circular agents.
	//@{
	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }
	//@}

	std::vector<SteerLib::AgentGoalInfo> m_allGoalsList;
	bool debug;
protected:
	/// Updates position, velocity, and orientation of the agent, given the force and dt time step.
	void _doEulerStep(const Util::Vector & steeringDecisionForce, float dt);

	virtual SteerLib::EngineInterface * getSimulationEngine();

	PotentialGrid *m_potentialGrid;
};

#endif
