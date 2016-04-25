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

	//std::cout << "making grid" << std::endl;

	float min_x = -15.0f; //gSpatialDatabase->getOriginX(); // this doesn't work
	float min_z = -15.0f; //gSpatialDatabase->getOriginZ();
	float max_x = 15.0f; //gSpatialDatabase->getGridSizeX() + min_x;
	float max_z = 15.0f; //gSpatialDatabase->getGridSizeZ() + min_z;

	//std::cout << "world bounds are min: " << min_x << " " << min_z << " max: " << max_x << " " << max_z << std::endl;

	m_densityVelocityGrid = new ContinuumGrid(RESOLUTION_X, RESOLUTION_Z, Util::Point(min_x, 0.0f, min_z), Util::Point(max_x, 0.0f, max_z));

	//std::cout << "finished making grid!" << std::endl;
}

void ContinuumModule::preprocessSimulation()
{
	//TODO does nothing for now
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
#if PRINTS
	std::cout << "made new agent!" << std::endl;
#endif
	ContinuumAgent *agent = new ContinuumAgent(m_densityVelocityGrid);
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

PotentialGrid::PotentialGrid(int res_x, int res_z, Util::Point min, Util::Point max, ContinuumGrid *speeds_densities)
{
	m_speeds_densities = speeds_densities;

	m_res_x = res_x;
	m_res_z = res_z;
	m_potential = new float_grid_2D(res_x, res_z, min, max);

	m_d_potential_N = new float_grid_2D(res_x, res_z, min, max);
	m_d_potential_S = new float_grid_2D(res_x, res_z, min, max);
	m_d_potential_E = new float_grid_2D(res_x, res_z, min, max);
	m_d_potential_W = new float_grid_2D(res_x, res_z, min, max);

	m_uCost_N = new float_grid_2D(res_x, res_z, min, max);
	m_uCost_S = new float_grid_2D(res_x, res_z, min, max);
	m_uCost_E = new float_grid_2D(res_x, res_z, min, max);
	m_uCost_W = new float_grid_2D(res_x, res_z, min, max);
	  
	m_speed_N = new float_grid_2D(res_x, res_z, min, max);
	m_speed_S = new float_grid_2D(res_x, res_z, min, max);
	m_speed_E = new float_grid_2D(res_x, res_z, min, max);
	m_speed_W = new float_grid_2D(res_x, res_z, min, max);

	m_known = new float_grid_2D(res_x, res_z, min, max);
}

PotentialGrid::~PotentialGrid()
{
	delete m_d_potential_N;
	delete m_d_potential_S;
	delete m_d_potential_E;
	delete m_d_potential_W;

	delete m_uCost_N;
	delete m_uCost_S;
	delete m_uCost_E;
	delete m_uCost_W;
			 
	delete m_speed_N;
	delete m_speed_S;
	delete m_speed_E;
	delete m_speed_W;

	delete m_known;
}

float speedSingleCell(ContinuumGrid *speed_density, bool eastWest, int i, int j)
{
	// sample the speed component at the cell specified
	// sample pressure at the cell specified
	// interpolate using density
	float fv;
	if (eastWest) {
		fv = speed_density->m_avg_vel_x->getByIndex(i, j);
	}
	else {
		fv = speed_density->m_avg_vel_z->getByIndex(i, j);
	}
	fv = max(fv, 0.0f); // clamp flow speed to be nonnegative
	float p = speed_density->m_density->getByIndex(i, j);

	float pmax = speed_density->m_max_density;
	//std::cout << "pmax is " << pmax << std::endl;
	//std::cout << "p is " << p << std::endl;
	//std::cout << "fv is " << fv << std::endl;
	//std::cout << "got " << ((p - DENSITY_MIN) / (pmax - DENSITY_MIN)) * fv << std::endl;
	return ((p - DENSITY_MIN) / (pmax - DENSITY_MIN)) * fv;
}

void PotentialGrid::computeSpeedField()
{
	// speed field computation
	// to get the speed value at cell x moving into a position p:
	// flow speed fv = vel(p) dot direction to P -> literally just sample a component of vel at p
	// speed field value f = (density(p) - pmin) / (pmax - pmin) * fv

	float speed_into_cell;
	// north
	for (int i = 0; i < m_res_x; i++) {
		for (int j = 0; j < m_res_z - 1; j++) {
			speed_into_cell = speedSingleCell(m_speeds_densities, false, i, j + 1);
			m_speed_N->setByIndex(i, j, speed_into_cell);
		}
	}
	// south
	for (int i = 0; i < m_res_x; i++) {
		for (int j = 1; j < m_res_z; j++) {
			speed_into_cell = speedSingleCell(m_speeds_densities, false, i, j - 1);
			m_speed_S->setByIndex(i, j, speed_into_cell);
		}
	}
	// east
	for (int i = 0; i < m_res_x - 1; i++) {
		for (int j = 0; j < m_res_z; j++) {
			speed_into_cell = speedSingleCell(m_speeds_densities, true, i + 1, j);
			m_speed_E->setByIndex(i, j, speed_into_cell);
		}
	}
	// west
	for (int i = 1; i < m_res_x; i++) {
		for (int j = 0; j < m_res_z; j++) {
			speed_into_cell = speedSingleCell(m_speeds_densities, true, i - 1, j);
			m_speed_W->setByIndex(i, j, speed_into_cell);
		}
	}
}

void unitCostSingleCell(float_grid_2D* speed_grid, float_grid_2D* cost_grid, int i, int j)
{
	float speed = speed_grid->getByIndex(i, j);
	float cost = speed * SPEED_WEIGHT + TIME_WEIGHT;
	if (abs(speed) < 0.0001f) {
		cost = TIME_WEIGHT;
		//std::cout << "zero speed detected!" << std::endl;
	}
	else {
		cost /= speed;
	}
	//std::cout << "cost at " << i << " " << j << " is " << cost << std::endl;
	cost_grid->setByIndex(i, j, cost);
}

void PotentialGrid::computeUnitCosts()
{
	for (int i = 0; i < m_res_x; i++) {
		for (int j = 0; j < m_res_z; j++) {
			unitCostSingleCell(m_speed_N, m_uCost_N, i, j);
			unitCostSingleCell(m_speed_S, m_uCost_S, i, j);
			unitCostSingleCell(m_speed_E, m_uCost_E, i, j);
			unitCostSingleCell(m_speed_W, m_uCost_W, i, j);
		}
	}
}

float quadForm(float a, float b, float c, float plusMinus) {
	plusMinus = plusMinus / abs(plusMinus);
	return (-b + plusMinus * sqrt(b * b - 4.0f * a * c)) / 2 * a;
}

float quadFormLarger(float a, float b, float c) {
	float plus = quadForm(a, b, c, 1.0f);
	float minu = quadForm(a, b, c, -1.0f);
	//std::cout << "a " << a << " b " << b << " c " << c << " minu " << minu << " plus " << plus << std::endl;
	return max(plus, minu);
}

float PotentialGrid::finiteDifference(int x, int z)
{
	// find least costly adjacent grid cell along both axes
	float minPX;
	float minPZ;
	float cX;
	float cZ;
	float candidatePotentialNE;
	float candidatePotentialSW;
	float candidateCostNE;
	float candidateCostSW;
	// out of bounds returns HUGE_VAL, so shouldn't be a problem here
	candidateCostSW = m_uCost_W->getByIndex(x, z);
	candidatePotentialSW = m_potential->getByIndex(x - 1, z);
	candidateCostNE = m_uCost_E->getByIndex(x, z);
	candidatePotentialNE = m_potential->getByIndex(x + 1, z);
	if ((candidatePotentialNE + candidateCostNE) < (candidatePotentialSW + candidateCostSW)) {
		minPX = candidatePotentialNE;
		cX = candidateCostNE;
	}
	else {
		minPX = candidatePotentialSW;
		cX = candidateCostSW;
	}

	candidateCostSW = m_uCost_S->getByIndex(x, z);
	candidatePotentialSW = m_potential->getByIndex(x, z - 1);
	candidateCostNE = m_uCost_N->getByIndex(x, z);
	candidatePotentialNE = m_potential->getByIndex(x, z + 1);
	if ((candidatePotentialNE + candidateCostNE) < (candidatePotentialSW + candidateCostSW)) {
		minPZ = candidatePotentialNE;
		cZ = candidateCostNE;
	}
	else {
		minPZ = candidatePotentialSW;
		cZ = candidateCostSW;
	}

	float a;
	float b;
	float c;
	float cZ2 = cZ * cZ;
	float cX2 = cX * cX;

	// solve for potential using quadform
	if (minPX < POTENTIAL_MAX && minPZ < POTENTIAL_MAX) {
		a = cX2 + cZ2;
		b = -2.0f * minPX * cZ2 - 2.0f * minPZ * cX2;
		c = cZ2 * minPX * minPX + cX2 * minPZ * minPZ - cX2 * cZ2;
	}
	else if (minPX < POTENTIAL_MAX) {
		a = 1.0f;
		b = -2.0f * minPX;
		c = minPX * minPX - cX2;
	}
	else {
		a = 1.0f;
		b = -2.0f * minPZ;
		c = minPZ * minPZ - cZ2;
	}
	float qForm = quadFormLarger(a, b, c);
	//std::cout << "cX " << cX << " cZ " << cZ << " a " << a << " b " << b << " c " << c << " qform " << qForm << std::endl;
	return qForm;
}

void PotentialGrid::addCandidates(std::priority_queue<cell_potential> &candidatesPQ, int x, int z)
{
	float finiteDiff;

	int numNeighbors = 0; // debug

	// do potentials of N, S, E, W surrounding this coordinate 
	// if the cell is in bounds, add to PQ
	// North
	if (m_potential->inBounds(x, z + 1) && m_known->getByIndex(x, z + 1) < 0.0f) {
		finiteDiff = finiteDifference(x, z + 1);
		cell_potential potential;
		potential.x = x;
		potential.z = z + 1;
		potential.potential = finiteDiff;
		candidatesPQ.push(potential);
		numNeighbors++;
	}
	// South
	if (m_potential->inBounds(x, z - 1) && m_known->getByIndex(x, z - 1) < 0.0f) {
		finiteDiff = finiteDifference(x, z - 1);
		cell_potential potential;
		potential.x = x;
		potential.z = z - 1;
		potential.potential = finiteDiff;
		candidatesPQ.push(potential);
		numNeighbors++;
	}
	// East
	if (m_potential->inBounds(x + 1, z) && m_known->getByIndex(x + 1, z) < 0.0f) {
		finiteDiff = finiteDifference(x + 1, z);
		cell_potential potential;
		potential.x = x + 1;
		potential.z = z;
		potential.potential = finiteDiff;
		candidatesPQ.push(potential);
		numNeighbors++;
	}
	// West
	if (m_potential->inBounds(x - 1, z) && m_known->getByIndex(x - 1, z) < 0.0f) {
		finiteDiff = finiteDifference(x - 1, z);
		cell_potential potential;
		potential.x = x - 1;
		potential.z = z;
		potential.potential = finiteDiff;
		candidatesPQ.push(potential);
		numNeighbors++;
	}
	//std::cout << "added " << numNeighbors << std::endl;
}

void PotentialGrid::splatGoal(Util::Point goalPosition)
{
	// http://stackoverflow.com/questions/9178083/priority-queue-for-user-defined-types
	std::priority_queue<cell_potential> candidatesPQ;
	int numCells = m_res_x * m_res_z;
	int numKnown = 0;

	int goal_pos_x;
	int goal_pos_z;
	m_potential->getIndicesForCoordinate(goalPosition.x, goalPosition.z, goal_pos_x, goal_pos_z);

	// begin by setting all potentials to infinity
	m_potential->clear(HUGE_VAL);
	// mark all cells as unknown: neg is unknown
	m_known->clear(-1.0f);
	// set goal potential to 0.0
	m_potential->setByIndex(goal_pos_x, goal_pos_z, 0.0f);
	// mark goal as "known"
	m_known->setByIndex(goal_pos_x, goal_pos_z, 10.0f);
	numKnown++;

	// approximate finite difference for cells adjacent to "known." Add to the "candidate" list.
	//std::cout << "adding first set of candidates" << std::endl;
	addCandidates(candidatesPQ, goal_pos_x, goal_pos_z);
	//std::cout << "entering flood fill loop" << std::endl;
	
	while (numKnown < numCells && !candidatesPQ.empty()) {
		// pop candidate with lowest potential diff from pq. if candidate is already known, pop another.
		//std::cout << "num candidates " << candidatesPQ.size() << std::endl;
		cell_potential lowest = candidatesPQ.top();
		//std::cout << "candidate val is " << lowest.potential;
		candidatesPQ.pop();
		if (std::isnan(lowest.potential)) {
			//std::cout << " continuing" << std::endl;
			continue;
		}
		//std::cout << " next candidate val is " << candidatesPQ.top().potential << std::endl;
		if (m_known->getByIndex(lowest.x, lowest.z) > 0.0f) continue;
		
		// update candidate's potential on the real grid
		m_potential->setByIndex(lowest.x, lowest.z, lowest.potential);
		// mark candidate as known, add neighbors to candidate set
		m_known->setByIndex(lowest.x, lowest.z, 10.0f);
		numKnown++;

		addCandidates(candidatesPQ, lowest.x, lowest.z);

		// done when all cells are known
	}

	// debug
	//m_potential->printGrid(goal_pos_x, goal_pos_z);

	// compute deltas
	computePotentialDeltas();
}

void PotentialGrid::computePotentialDeltas()
{
	// speed field computation
	// to get the speed value at cell x moving into a position p:
	// flow speed fv = vel(p) dot direction to P -> literally just sample a component of vel at p
	// speed field value f = (density(p) - pmin) / (pmax - pmin) * fv

	float sample_1;
	float sample_2;
	// north
	for (int i = 0; i < m_res_x; i++) {
		for (int j = 0; j < m_res_z - 1; j++) {
			sample_1 = m_potential->getByIndex(i, j);
			sample_2 = m_potential->getByIndex(i, j + 1);
			m_d_potential_N->setByIndex(i, j, sample_2 - sample_1);
		}
	}
	// south
	for (int i = 0; i < m_res_x; i++) {
		for (int j = 1; j < m_res_z; j++) {
			sample_1 = m_potential->getByIndex(i, j);
			sample_2 = m_potential->getByIndex(i, j - 1);
			m_d_potential_S->setByIndex(i, j, sample_2 - sample_1);
		}
	}
	// east
	for (int i = 0; i < m_res_x - 1; i++) {
		for (int j = 0; j < m_res_z; j++) {
			sample_1 = m_potential->getByIndex(i, j);
			sample_2 = m_potential->getByIndex(i + 1, j);
			m_d_potential_E->setByIndex(i, j, sample_2 - sample_1);
		}
	}
	// west
	for (int i = 1; i < m_res_x; i++) {
		for (int j = 0; j < m_res_z; j++) {
			sample_1 = m_potential->getByIndex(i, j);
			sample_2 = m_potential->getByIndex(i - 1, j);
			m_d_potential_W->setByIndex(i, j, sample_2 - sample_1);
		}
	}
}

void PotentialGrid::update(Util::Point goalPosition)
{
	computeSpeedField();

	computeUnitCosts();
	//std::cout << "splatting goal" << std::endl;
	splatGoal(goalPosition);
}

ContinuumGrid::ContinuumGrid(int res_x, int res_z, Util::Point min, Util::Point max)
{
	m_density = new float_grid_2D(res_x, res_z, min, max);
	m_avg_vel_x = new float_grid_2D(res_x, res_z, min, max);
	m_avg_vel_z = new float_grid_2D(res_x, res_z, min, max);
	m_res_x = res_x;
	m_res_z = res_z;
	m_min = min;
	m_max = max;
}

ContinuumGrid::~ContinuumGrid()
{
	delete m_density;
	delete m_avg_vel_x;
	delete m_avg_vel_z;
}

void ContinuumGrid::reset()
{
	m_density->clear(0.0f);
	m_avg_vel_x->clear(0.0f);
	m_avg_vel_z->clear(0.0f);
}

void ContinuumGrid::splatAgent(Util::Point agentPosition, Util::Vector agentVelocity)
{
	// get the position of the closest cell with coordinates < agentPosition
	float x_cell;
	float z_cell;
	m_density->cellCenter(agentPosition.x, agentPosition.z, x_cell, z_cell);
	if (x_cell > agentPosition.x) x_cell -= m_density->m_cell_size_x;
	if (z_cell > agentPosition.z) z_cell -= m_density->m_cell_size_z;

	int x_cell_idx;
	int z_cell_idx;
	m_density->getIndicesForCoordinate(x_cell, z_cell, x_cell_idx, z_cell_idx);

	// compute contribs to this cell and each cell w/larger coord
	float cell_width = m_density->m_cell_size_x;
	float cell_height = m_density->m_cell_size_z;

	float dx = abs(agentPosition.x - x_cell) / cell_width;
	float dz = abs(agentPosition.z - z_cell) / cell_height;

	float pA = pow(min(1.0f - dx, 1.0f - dz), DENSITY_FALLOFF); // this cell
	float pB = pow(min(dx, 1.0f - dz), DENSITY_FALLOFF); // right
	float pC = pow(min(dx,dz), DENSITY_FALLOFF); // up-right
	float pD = pow(min(1.0f - dx, dz), DENSITY_FALLOFF); // up

	// add contributions to each cell with coordinates larger
	m_density->addByIndex(x_cell_idx, z_cell_idx, pA);
	m_density->addByIndex(x_cell_idx + 1, z_cell_idx, pB);
	m_density->addByIndex(x_cell_idx + 1, z_cell_idx + 1, pC);
	m_density->addByIndex(x_cell_idx, z_cell_idx + 1, pD);

	// compute avg velocity contributions
	float vAx = agentVelocity.x * pA;
	float vBx = agentVelocity.x * pB;
	float vCx = agentVelocity.x * pC;
	float vDx = agentVelocity.x * pD;

	float vAz = agentVelocity.z * pA;
	float vBz = agentVelocity.z * pB;
	float vCz = agentVelocity.z * pC;
	float vDz = agentVelocity.z * pD;

	m_avg_vel_x->addByIndex(x_cell_idx, z_cell_idx, vAx);
	m_avg_vel_x->addByIndex(x_cell_idx + 1, z_cell_idx, vBx);
	m_avg_vel_x->addByIndex(x_cell_idx + 1, z_cell_idx + 1, vCx);
	m_avg_vel_x->addByIndex(x_cell_idx, z_cell_idx + 1, vDx);

	m_avg_vel_z->addByIndex(x_cell_idx, z_cell_idx, vAz);
	m_avg_vel_z->addByIndex(x_cell_idx + 1, z_cell_idx, vBz);
	m_avg_vel_z->addByIndex(x_cell_idx + 1, z_cell_idx + 1, vCz);
	m_avg_vel_z->addByIndex(x_cell_idx, z_cell_idx + 1, vDz);
}

void ContinuumGrid::normalizeVelocitiesByDensity()
{
	float avg_density;
	float avg_x;
	float avg_z;
	m_max_density = 0.0f;
	for (int i = 0; i < m_res_x; i++) {
		for (int j = 0; j < m_res_z; j++) {
			avg_density = m_density->getByIndex(i, j);
			m_max_density = max(avg_density, m_max_density);
			avg_x = m_avg_vel_x->getByIndex(i, j) / avg_density;
			avg_z = m_avg_vel_z->getByIndex(i, j) / avg_density;
			m_avg_vel_x->setByIndex(i, j, avg_x);
			m_avg_vel_z->setByIndex(i, j, avg_z);
		}
	}
}

float_grid_2D::float_grid_2D(int res_x, int res_z, Util::Point min, Util::Point max)
{
	m_res_x = res_x;
	m_res_z = res_z;
	m_min = min;
	m_max = max;
	for (int i = 0; i < res_x; i++) {
		m_values.push_back(std::vector<float>());
	}

	for (int i = 0; i < res_x; i++) {
		for (int j = 0; j < res_z; j++) {
			m_values.at(i).push_back(0.0f);
		}
	}
	m_cell_size_x = (m_max.x - m_min.x) / m_res_x;
	m_cell_size_z = (m_max.z - m_min.z) / m_res_z;
}

float_grid_2D::~float_grid_2D()
{
}

float float_grid_2D::getByIndex(int x, int z)
{
	// for m_res_z = 3 and m _res_x = 4:
	//   0  1  2  3
	// 0 0  1  2  3
	// 1 4  5  6  7
	// 2 8  9  10 11
	if (x < 0 || x >= m_res_x || z < 0 || z >= m_res_z) return HUGE_VAL;
	return m_values.at(x).at(z);
}

void float_grid_2D::setByIndex(int x, int z, float val)
{
	if (x < 0 || x >= m_res_x || z < 0 || z >= m_res_z) return;
	m_values.at(x).at(z) = val;
}

void float_grid_2D::addByIndex(int x, int z, float val)
{
	if (x < 0 || x >= m_res_x || z < 0 || z >= m_res_z) return;
	m_values.at(x).at(z) += val;
}

void float_grid_2D::clear(float val)
{
	for (int i = 0; i < m_res_x; i++) {
		for (int j = 0; j < m_res_z; j++) {
			m_values.at(i).at(j) = val;
		}
	}
}

float float_grid_2D::getByCoordinate(float x, float z)
{
	int x_idx;
	int z_idx;
	getIndicesForCoordinate(x, z, x_idx, z_idx);

	return getByIndex(x_idx, z_idx);
}

void float_grid_2D::setByCoordinate(float x, float z, float val)
{
	int x_idx;
	int z_idx;
	getIndicesForCoordinate(x, z, x_idx, z_idx);

	setByIndex(x_idx, z_idx, val);
}

void float_grid_2D::getIndicesForCoordinate(float x, float z, int &x_cell, int &z_cell)
{
	float coord_localized_x = x - m_min.x;
	float coord_localized_z = z - m_min.z;

	//   0  1  2  3
	// 0 0  1  2  3
	// 1 4  5  6  7
	// 2 8  9  10 11

	// min is considered to be the top left corner of this grid
	// max is considered to be the bottom right corner of this grid
	// convert x and z into cell indices

	x_cell = int(coord_localized_x / m_cell_size_x);
	z_cell = int(coord_localized_z / m_cell_size_z);
}

void float_grid_2D::cellCenter(float x, float z, float &x_cell, float &z_cell)
{
	// return the coordinates of the cell center that this cell is in

	int x_idx;
	int z_idx;
	getIndicesForCoordinate(x, z, x_idx, z_idx);

	// now get coordinates of cell center
	// coordinates computed will be the "least" corner of the cell
	// also need to delocalize
	x_cell = x_idx * m_cell_size_x + m_cell_size_x * 0.5f + m_min.x;
	z_cell = z_idx * m_cell_size_z + m_cell_size_z * 0.5f + m_min.z;
}

bool float_grid_2D::inBounds(int x, int z)
{
	return (x >= 0 && x < m_res_x && z >= 0 && z < m_res_z);
}

void float_grid_2D::printGrid(int x, int z)
{
	for (int i = 0; i < m_res_x; i++) {
		for (int j = 0; j < m_res_z; j++) {
			if (i == x && j == z) {
				std::cout << "{" << (int)(10 * m_values.at(i).at(j)) << "}";
				continue;
			}
			if (m_values.at(i).at(j) > 0.001f)
				std::cout << "[" << (int) (10 * m_values.at(i).at(j)) << "]";//m_values.at(i).at(j) << " ";
			else std::cout << "[ ]";
		}
		std::cout << std::endl;
	}
}