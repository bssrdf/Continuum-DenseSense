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
#include "floatGrid2D.h"
#include "continuumGrid.h"

class ContinuumAgent; // forward declaration

#define RESOLUTION_X 30
#define RESOLUTION_Z 30

// globally accessible to the continuum plugin
extern SteerLib::EngineInterface * gEngine;
extern SteerLib::SpatialDataBaseInterface * gSpatialDatabase;

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