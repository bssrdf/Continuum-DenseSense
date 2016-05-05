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
#if _DEBUG
	std::cout << "preprocessing sim..." << std::endl;
#endif
	//Build grids that span everything in the scene

	//std::cout << "making grid" << std::endl;

	float min_x = -25.0f;// gSpatialDatabase->getOriginX(); // this doesn't REALLY work. fix it!
	float min_z = -25.0f;// gSpatialDatabase->getOriginZ();
	float max_x = 25.0f;// gSpatialDatabase->getGridSizeX() + min_x;
	float max_z = 25.0f;// gSpatialDatabase->getGridSizeZ() + min_z;

	std::cout << "world bounds are min: " << min_x << " " << min_z << " max: " << max_x << " " << max_z << std::endl;

	m_densityVelocityGrid = new ContinuumGrid(RESOLUTION_X, RESOLUTION_Z, Util::Point(min_x, 0.0f, min_z), Util::Point(max_x, 0.0f, max_z));

	//std::cout << "finished making grid!" << std::endl;

	// initialize all the agents properly
	int num_agents = m_agents.size();
	for (int i = 0; i < num_agents; i++) {
		m_agents[i]->init(m_densityVelocityGrid);
	}


#if _DEBUG
	std::cout << "done preprocessing" << std::endl;
#endif
}

void ContinuumModule::preprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	//std::cout << "preprocess frame..." << std::endl;
	// splat all the agents
	m_densityVelocityGrid->reset();
	int num_agents = m_agents.size();
	for (int i = 0; i < num_agents; i++) {
		ContinuumAgent *agent = m_agents.at(i);
		if (agent->enabled()) {
			m_densityVelocityGrid->splatAgent(agent->position(), agent->velocity());
		}
	}
	m_densityVelocityGrid->normalizeVelocitiesByDensity();
	//m_densityVelocityGrid->m_density->printGrid(-1, -1);
	//std::cout << "preprocess complete!" << std::endl;
}

void ContinuumModule::postprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	//TODO does nothing for now
#if PRINTS
	std::cout << "postprocess frame..." << std::endl;
#endif
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