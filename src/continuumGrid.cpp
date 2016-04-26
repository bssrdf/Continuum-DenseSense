#include "continuumGrid.h"

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
