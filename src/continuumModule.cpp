//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


/// @file SimpleAIModule.cpp
/// @brief Implements the SimpleAIModule plugin.


#include "SteerLib.h"
#include "SimulationPlugin.h"
#include "continuumModule.h"
#include "continuumAgent.h"

#include "LogObject.h"
#include "LogManager.h"

#define PRINTS 0

// globally accessible to the simpleAI plugin
SteerLib::EngineInterface * gEngine;
SteerLib::SpatialDataBaseInterface * gSpatialDatabase;

namespace ContinuumGlobals
{
	unsigned int gLongTermPlanningPhaseInterval;
	unsigned int gMidTermPlanningPhaseInterval;
	unsigned int gShortTermPlanningPhaseInterval;
	unsigned int gPredictivePhaseInterval;
	unsigned int gReactivePhaseInterval;
	unsigned int gPerceptivePhaseInterval;
	bool gUseDynamicPhaseScheduling;
	bool gShowStats;
	bool gShowAllStats;

	PhaseProfilers * gPhaseProfilers;
}

using namespace ContinuumGlobals;

PLUGIN_API SteerLib::ModuleInterface * createModule()
{
	return new ContinuumModule;
}

PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module )
{
	delete module;
}


void ContinuumModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
{
	m_densityVelocityGrid = NULL;

	gEngine = engineInfo;
	gSpatialDatabase = engineInfo->getSpatialDatabase();
#if PRINTS
	std::cout << "spatial database size x" << gSpatialDatabase->getGridSizeX() << std::endl;
	std::cout << "spatial database size z" << gSpatialDatabase->getGridSizeZ() << std::endl;
#endif

	gUseDynamicPhaseScheduling = false;
	gShowStats = false;
	logStats = false;
	gShowAllStats = false;
	logFilename = "continuum.log";

	SteerLib::OptionDictionary::const_iterator optionIter;
	for (optionIter = options.begin(); optionIter != options.end(); ++optionIter) {
		std::stringstream value((*optionIter).second);
		if ((*optionIter).first == "")
		{
			value >> gLongTermPlanningPhaseInterval;
		}
		else if ((*optionIter).first == "ailogFileName")
		{
			logFilename = value.str();
			logStats = true;
		}
		else if ((*optionIter).first == "stats")
		{
			gShowStats = Util::getBoolFromString(value.str());
		}
		else if ((*optionIter).first == "allstats")
		{
			gShowAllStats = Util::getBoolFromString(value.str());
		}
		else
		{
			// throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to PPR AI module.");
		}
	}

	if( logStats )
	{

		_logger = LogManager::getInstance()->createLogger(logFilename,LoggerType::BASIC_WRITE);

		_logger->addDataField("number_of_times_executed",DataType::LongLong );
		_logger->addDataField("total_ticks_accumulated",DataType::LongLong );
		_logger->addDataField("shortest_execution",DataType::LongLong );
		_logger->addDataField("longest_execution",DataType::LongLong );
		_logger->addDataField("fastest_execution", DataType::Float);
		_logger->addDataField("slowest_execution", DataType::Float);
		_logger->addDataField("average_time_per_call", DataType::Float);
		_logger->addDataField("total_time_of_all_calls", DataType::Float);
		_logger->addDataField("tick_frequency", DataType::Float);

		// LETS TRY TO WRITE THE LABELS OF EACH FIELD
		std::stringstream labelStream;
		unsigned int i;
		for (i=0; i < _logger->getNumberOfFields() - 1; i++)
			labelStream << _logger->getFieldName(i) << " ";
		labelStream << _logger->getFieldName(i);

		_logger->writeData(labelStream.str());

	}
}

void ContinuumModule::initializeSimulation()
{
	//
	// initialize the performance profilers
	//
	gPhaseProfilers = new PhaseProfilers;
	gPhaseProfilers->aiProfiler.reset();
	gPhaseProfilers->longTermPhaseProfiler.reset();
	gPhaseProfilers->midTermPhaseProfiler.reset();
	gPhaseProfilers->shortTermPhaseProfiler.reset();
	gPhaseProfilers->perceptivePhaseProfiler.reset();
	gPhaseProfilers->predictivePhaseProfiler.reset();
	gPhaseProfilers->reactivePhaseProfiler.reset();
	gPhaseProfilers->steeringPhaseProfiler.reset();
}

void ContinuumModule::preprocessSimulation()
{

}

void ContinuumModule::setupGrids() {
	//Build grids that are double the size needed to span everything in the scene
	// make sure the resolution is sufficient so that cell is at least two agents wide

	std::set<SteerLib::SpatialDatabaseItemPtr> neighborList;
	float big_val = HUGE_VAL;
	gSpatialDatabase->getItemsInRange(neighborList, -big_val, big_val, -big_val, big_val, NULL);


	SteerLib::ObstacleInterface *tmp_ob;
	SteerLib::AgentInterface * tmp_agent;
	Util::AxisAlignedBox tmp_bound;

	Util::Point worldMin;
	worldMin.x = big_val;
	worldMin.z = big_val;
	Util::Point worldMax;
	worldMax.x = -big_val;
	worldMax.z = -big_val;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = neighborList.begin(); 
		neighbour != neighborList.end(); neighbour++)
	{
		if (!(*neighbour)->isAgent())
		{
			// check the obstacle's AABB
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
			tmp_bound = tmp_ob->getBounds();
			worldMin.x = std::fmin(tmp_bound.xmin, worldMin.x);
			worldMin.z = std::fmin(tmp_bound.zmin, worldMin.z);
			worldMax.x = std::fmax(tmp_bound.xmax, worldMax.x);
			worldMax.z = std::fmax(tmp_bound.zmax, worldMax.z);
		}
		else {
			// check the agent's position
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);
			worldMin.x = std::fmin(tmp_agent->position().x, worldMin.x);
			worldMin.z = std::fmin(tmp_agent->position().z, worldMin.z);
			worldMax.x = std::fmax(tmp_agent->position().x, worldMax.x);
			worldMax.z = std::fmax(tmp_agent->position().z, worldMax.z);
		}
	}

	// check positions and goals of agents we know about
	int numAgents = m_agents.size();
	ContinuumAgent *tmp_cAgent;
	int tmp_numGoals;
	SteerLib::AgentGoalInfo tmp_goal;
	float biggestAgentRad = 0.0f;

	for (int i = 0; i < numAgents; i++) {
		tmp_cAgent = m_agents[i];
		biggestAgentRad = std::fmax(tmp_cAgent->radius(), biggestAgentRad);
		worldMin.x = std::fmin(tmp_cAgent->position().x, worldMin.x);
		worldMin.z = std::fmin(tmp_cAgent->position().z, worldMin.z);
		worldMax.x = std::fmax(tmp_cAgent->position().x, worldMax.x);
		worldMax.z = std::fmax(tmp_cAgent->position().z, worldMax.z);

		tmp_numGoals = tmp_cAgent->m_allGoalsList.size();
		for (int i = 0; i < tmp_numGoals; i++) {
			tmp_goal = tmp_cAgent->m_allGoalsList[i];
			if (tmp_goal.goalType == SteerLib::GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL) {
				worldMin.x = std::fmin(tmp_goal.targetRegion.xmin, worldMin.x);
				worldMin.z = std::fmin(tmp_goal.targetRegion.zmin, worldMin.z);
				worldMax.x = std::fmax(tmp_goal.targetRegion.xmax, worldMax.x);
				worldMax.z = std::fmax(tmp_goal.targetRegion.zmax, worldMax.z);
			}
			else {
				worldMin.x = std::fmin(tmp_goal.targetLocation.x, worldMin.x);
				worldMin.z = std::fmin(tmp_goal.targetLocation.z, worldMin.z);
				worldMax.x = std::fmax(tmp_goal.targetLocation.x, worldMax.x);
				worldMax.z = std::fmax(tmp_goal.targetLocation.z, worldMax.z);
			}
		}
	}

	std::cout << "all obstacles, agents, and goals in the sim are within";
	std::cout << " min: " << worldMin.x << " " << worldMin.z;
	std::cout << " max: " << worldMax.x << " " << worldMax.z << std::endl;

	// pad grid size
	float distX = worldMax.x - worldMin.x;
	worldMax.x += distX / 4.0f;
	worldMin.x -= distX / 4.0f;
	distX *= 1.5f;

	float distZ = worldMax.z - worldMin.z;
	worldMax.z += distZ / 4.0f;
	worldMin.z -= distZ / 4.0f;
	distZ *= 1.5f;

	// compute resolution so the biggest agent is at most half the cell size in either direction
	int resX = distX / (biggestAgentRad * 4.0f);
	int resZ = distZ / (biggestAgentRad * 4.0f);

	if (resX < 0) resX = 1;
	if (resZ < 0) resZ = 1;

	std::cout << "selected resolution is " << resX << " " << resZ << std::endl;

	m_densityVelocityGrid = new ContinuumGrid(resX, resZ, worldMin, worldMax);

	//std::cout << "finished making grid!" << std::endl;

	// initialize all the agents properly
	int num_agents = m_agents.size();
	for (int i = 0; i < num_agents; i++) {
		m_agents[i]->init(m_densityVelocityGrid);
	}
}

void ContinuumModule::preprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	if (m_densityVelocityGrid == NULL) { // on first frame, set up grids
		setupGrids();
	}

	// splat all the agents
	m_densityVelocityGrid->resetSplats();
	int num_agents = m_agents.size();
	for (int i = 0; i < num_agents; i++) {
		ContinuumAgent *agent = m_agents.at(i);
		if (agent->enabled()) {
			m_densityVelocityGrid->splatAgent(agent->position(), agent->velocity());
		}
	}
	m_densityVelocityGrid->normalizeVelocitiesByDensity();
}

void ContinuumModule::postprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{

}

void ContinuumModule::cleanupSimulation()
{
	delete m_densityVelocityGrid;

	if ( logStats )
	{
		LogObject logObject;

		logObject.addLogData(gPhaseProfilers->aiProfiler.getNumTimesExecuted());
		logObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTicksAccumulated());
		logObject.addLogData(gPhaseProfilers->aiProfiler.getMinTicks());
		logObject.addLogData(gPhaseProfilers->aiProfiler.getMaxTicks());
		logObject.addLogData(gPhaseProfilers->aiProfiler.getMinExecutionTimeMills());
		logObject.addLogData(gPhaseProfilers->aiProfiler.getMaxExecutionTimeMills());
		logObject.addLogData(gPhaseProfilers->aiProfiler.getAverageExecutionTimeMills());
		logObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTime());
		logObject.addLogData(gPhaseProfilers->aiProfiler.getTickFrequency());

		_logger->writeLogObject(logObject);

		// cleanup profileing metrics for next simulation/scenario
		gPhaseProfilers->aiProfiler.reset();
		gPhaseProfilers->longTermPhaseProfiler.reset();
		gPhaseProfilers->midTermPhaseProfiler.reset();
		gPhaseProfilers->shortTermPhaseProfiler.reset();
		gPhaseProfilers->perceptivePhaseProfiler.reset();
		gPhaseProfilers->predictivePhaseProfiler.reset();
		gPhaseProfilers->reactivePhaseProfiler.reset();
		gPhaseProfilers->steeringPhaseProfiler.reset();
	}

	// kdTree_->deleteObstacleTree(kdTree_->obstacleTree_);
}

void ContinuumModule::finish()
{
	// nothing to do here
}

SteerLib::AgentInterface * ContinuumModule::createAgent()
{
#if _DEBUG
	std::cout << "made new agent!" << std::endl;
#endif

	ContinuumAgent *agent = new ContinuumAgent();
	m_agents.push_back(agent);
	return agent;
}

void ContinuumModule::destroyAgent( SteerLib::AgentInterface * agent )
{
	int num_agents = m_agents.size();
	for (int i = 0; i < num_agents; i++) {
		if (m_agents.at(i) == agent) {
			m_agents.erase(m_agents.begin() + i);
			break;
		}
	}
	delete agent;
}