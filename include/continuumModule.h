//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#ifndef __CONTINUUM_MODULE__
#define __CONTINUUM_MODULE__

/// @file continuumModule.h
/// @brief Declares the continuumModule plugin.


#include "SteerLib.h"
#include "Logger.h"
#include "continuumAgent.h"

#define DENSITY_FALLOFF 0.9f
#define TIME_WEIGHT 0.7f
#define SPEED_WEIGHT 0.9f
#define DENSITY_MIN 0.001f
#define POTENTIAL_MAX 1000.0f
#define RESOLUTION_X 20
#define RESOLUTION_Z 20

// globally accessible to the continuum plugin
extern SteerLib::EngineInterface * gEngine;
extern SteerLib::SpatialDataBaseInterface * gSpatialDatabase;

class ContinuumAgent;

namespace ContinuumGlobals {

	struct PhaseProfilers {
		Util::PerformanceProfiler aiProfiler;
		Util::PerformanceProfiler drawProfiler;
		Util::PerformanceProfiler longTermPhaseProfiler;
		Util::PerformanceProfiler midTermPhaseProfiler;
		Util::PerformanceProfiler shortTermPhaseProfiler;
		Util::PerformanceProfiler perceptivePhaseProfiler;
		Util::PerformanceProfiler predictivePhaseProfiler;
		Util::PerformanceProfiler reactivePhaseProfiler;
		Util::PerformanceProfiler steeringPhaseProfiler;
	};


	extern unsigned int gLongTermPlanningPhaseInterval;
	extern unsigned int gMidTermPlanningPhaseInterval;
	extern unsigned int gShortTermPlanningPhaseInterval;
	extern unsigned int gPredictivePhaseInterval;
	extern unsigned int gReactivePhaseInterval;
	extern unsigned int gPerceptivePhaseInterval;
	extern bool gUseDynamicPhaseScheduling;
	extern bool gShowStats;
	extern bool gShowAllStats;

	extern PhaseProfilers * gPhaseProfilers;
}

class float_grid_2D{
public:
	float_grid_2D(int res_x, int res_z, Util::Point min, Util::Point max);
	~float_grid_2D();
	float getByIndex(int x, int z);
	void setByIndex(int x, int z, float val);
	void addByIndex(int x, int z, float val);
	void clear(float val);

	// grid is m_res_x by m_res_z between m_min and m_max
	float getByCoordinate(float x, float z);
	void setByCoordinate(float x, float z, float val);

	void getIndicesForCoordinate(float x, float z, int &x_cell, int &z_cell);
	void cellCenter(float x, float z, float &x_cell, float &z_cell);
	void printGrid(int x, int z);

	bool inBounds(int x, int z);

	std::vector<std::vector<float>> m_values;
	int m_res_x;
	int m_res_z;
	float m_cell_size_x;
	float m_cell_size_z;
	Util::Point m_min;
	Util::Point m_max;
};

class ContinuumGrid
{
	// MAC grid for all other values
	// for now, ignoring:
	// - height
	// - discomfort -> to be added
	// - MAC part for velocity -> inaccurate, but acceptable
public:
	// center values
	float_grid_2D *m_density;
	float_grid_2D *m_avg_vel_x; // sum(p_i * vx_i) / p
	float_grid_2D *m_avg_vel_z; // sum(p_i * vz_i) / p
	int m_res_x;
	int m_res_z;
	float m_max_density;
	Util::Point m_min;
	Util::Point m_max;

	ContinuumGrid(int res_x, int res_z, Util::Point min, Util::Point max);
	~ContinuumGrid();

	void reset(); // clear the grid
	void splatAgent(Util::Point agentPosition, Util::Vector agentVelocity);
	void normalizeVelocitiesByDensity();

};

struct cell_potential{
	int x;
	int z;
	float potential;
	bool operator<(const cell_potential& b) const{
		return potential > b.potential;
	}
};

class PotentialGrid
{
	// each agent gets its own potential grid for the time being
	// MAC grid for potentials
public:
	// center values
	float_grid_2D *m_potential;
	float_grid_2D *m_known;
	// staggered in between cells. ONLY access these by index!
	// yes, some values will be duplicated, but it'll just be easier to handle this way
	float_grid_2D *m_d_potential_N;
	float_grid_2D *m_d_potential_S;
	float_grid_2D *m_d_potential_E;
	float_grid_2D *m_d_potential_W;

	// we need to double stagger b/c we care about both directions here
	float_grid_2D *m_speed_N;
	float_grid_2D *m_speed_S;
	float_grid_2D *m_speed_E;
	float_grid_2D *m_speed_W;

	float_grid_2D *m_uCost_N;
	float_grid_2D *m_uCost_S;
	float_grid_2D *m_uCost_E;
	float_grid_2D *m_uCost_W;

	int m_res_x;
	int m_res_z;

	ContinuumGrid *m_speeds_densities;

	PotentialGrid(int res_x, int res_z, Util::Point min, Util::Point max, ContinuumGrid *speeds_densities);
	~PotentialGrid();

	void update(Util::Point goalPosition);

private:
	float finiteDifference(int x, int z);
	void computeSpeedField();
	void computeUnitCosts();
	void addCandidates(std::priority_queue<cell_potential> &candidatesPQ, int x, int z);
	void splatGoal(Util::Point goalPosition); // handle potential changes as a result of adding a goal

	void computePotentialDeltas(); // called by splatGoal: no need to call this on its own
};


/**
 * @brief An example plugin for the SimulationEngine that provides very basic AI agents.
 *
 * This class is an example of a plug-in module (as opposed to a built-in module).
 * It compiles as part of a dynamic library which is loaded by a SimulationEngine at run-time.
 *
 * The simpleAI plugin consists of three parts:
 *  - This class inherits from SteerLib::ModuleInterface, and implements only the desired functionality.  In this case
 *    the desired functionality is to be able to create/destroy SimpleAgent agents.
 *  - The two global functions createModule() and destroyModule() are implemented so that the engine can load the
 *    dynamic library and get an instance of our SimpleAIModule.
 *  - The SimpleAgent class inherits from SteerLib::AgentInterface, which is the agent steering AI used by the engine.
 *    this agent serves as an example of how to create your own steering AI using SteerLib features.
 *
 */



/**
Algorithm:
-we're going to make each agent its own group (blaw)
  -b/c test cases might not allow otherwise?
  -we may be able to preprocess agents by goal, but -> after Thurs
-requires a 2D MAC Grid

-each timestep:
  -module preprocess: snapshot of current situation
    -build global density field on mac grid (section 4.1)
    -build global average velocity field on mac grid (equation 7)

  -module preprocess: computing new optimal paths for each group
    -for each agent, build speed field for agent (section 4.2 and equation 10, remove fT terms)
      -needs: density at some point where agent wants to go
      -needs: density/average velocity fields from earlier

    -for each agent, build potential field -> section 4.3
      -TODO
      -involves a kind of "flood fill" using a heap (or just a queue)
      -since we don't have heights here, can we just use radial distance from the goal?


  -agent process:
    -sample personal speed and density grids where agent is now to get a velocity -> end of section 4.3
    -update locations -> tweak what simpleAI already gives

-location enforcement
  -bin all agents onto a neighbor grid of cell size r
  -at each grid
    -for each agent
        -check 9 grids centered around this one for neighbors
        -move self and neighbor if any are closer than r
 */

class ContinuumModule : public SteerLib::ModuleInterface
{
public:
	
	//std::string getDependencies() { return "testCasePlayer"; }
	std::string getDependencies() { return ""; }
	
	std::string getConflicts() { return ""; }
	std::string getData() { return ""; }
	LogData * getLogData() { return new LogData(); }
	void init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo );
	void finish();
	SteerLib::AgentInterface * createAgent();
	void destroyAgent( SteerLib::AgentInterface * agent );

	void preprocessFrame(float timeStamp, float dt, unsigned int frameNumber);
	void postprocessFrame(float timeStamp, float dt, unsigned int frameNumber);
	void preprocessSimulation();
	void initializeSimulation();
	void cleanupSimulation();

protected:
	std::string logFilename; // = "AI.log";
	bool logStats; // = false;
	Logger * _logger;

	ContinuumGrid *m_densityVelocityGrid;
	std::vector<ContinuumAgent*> m_agents;
};
#endif