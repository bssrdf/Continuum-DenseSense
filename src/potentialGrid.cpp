#include "potentialGrid.hpp"


PotentialGrid::PotentialGrid(int res_x, int res_z, Util::Point min, Util::Point max, ContinuumGrid *speeds_densities)
{
	m_speeds_densities = speeds_densities;

	m_res_x = res_x;
	m_res_z = res_z;
	m_potential = new float_grid_2D(res_x, res_z, min, max);

	m_dPotential_N = new float_grid_2D(res_x, res_z, min, max);
	m_dPotential_S = new float_grid_2D(res_x, res_z, min, max);
	m_dPotential_E = new float_grid_2D(res_x, res_z, min, max);
	m_dPotential_W = new float_grid_2D(res_x, res_z, min, max);

	m_velocity_N = new float_grid_2D(res_x, res_z, min, max);
	m_velocity_S = new float_grid_2D(res_x, res_z, min, max);
	m_velocity_E = new float_grid_2D(res_x, res_z, min, max);
	m_velocity_W = new float_grid_2D(res_x, res_z, min, max);

	m_known = new float_grid_2D(res_x, res_z, min, max);
}

PotentialGrid::~PotentialGrid()
{

	delete m_potential;

	delete m_dPotential_N;
	delete m_dPotential_S;
	delete m_dPotential_E;
	delete m_dPotential_W;

	delete m_velocity_N;
	delete m_velocity_S;
	delete m_velocity_E;
	delete m_velocity_W;

	delete m_known;
}

float quadForm(float a, float b, float c, float plusMinus) {
	//plusMinus = plusMinus / abs(plusMinus);
	return (-b + plusMinus * sqrt(b * b - 4.0f * a * c)) / (2.0f * a);
}

float quadFormLarger(float a, float b, float c) {
	float plus = quadForm(a, b, c, 1.0f);
	float minu = quadForm(a, b, c, -1.0f);
	if (std::isnan(plus)) return minu;
	if (std::isnan(minu)) return plus;

	return max(plus, minu);
}

float PotentialGrid::finiteDifference(int x, int z)
{
	// find least costly adjacent grid cell along both axes
	float candidatePotentialN;
	float candidatePotentialS;
	float candidatePotentialE;
	float candidatePotentialW;

	float candidateCostN;
	float candidateCostS;
	float candidateCostE;
	float candidateCostW;

	float potentialZ;
	float potentialX;
	float costZ;
	float costX;

	candidateCostW = m_speeds_densities->m_cost_W->getByIndex(x - 1, z);
	candidatePotentialW = m_potential->getByIndex(x - 1, z);

	candidateCostE = m_speeds_densities->m_cost_E->getByIndex(x + 1, z);
	candidatePotentialE = m_potential->getByIndex(x + 1, z);

	// out of bounds for potential is huge val, so that shouldn't be a problem
	if ((candidateCostW + candidatePotentialW) < (candidateCostE + candidatePotentialE)) {
		potentialZ = candidatePotentialW;
		costZ = candidateCostW;
	}
	else {
		potentialZ = candidatePotentialE;
		costZ = candidateCostE;
	}

	candidateCostS = m_speeds_densities->m_cost_S->getByIndex(x, z - 1);
	candidatePotentialS = m_potential->getByIndex(x, z - 1);
	candidateCostN = m_speeds_densities->m_cost_N->getByIndex(x, z + 1);
	candidatePotentialN = m_potential->getByIndex(x, z + 1);
	if ((candidatePotentialN + candidateCostN) < (candidatePotentialS + candidateCostS)) {
		potentialX = candidatePotentialN;
		costX = candidateCostN;
	}
	else {
		potentialX = candidatePotentialS;
		costX = candidateCostS;
	}

	float a;
	float b;
	float c;
	float cZ2 = costZ * costZ;
	float cX2 = costX * costX;

	// solve for potential using quadform
	// drop infinite terms
	if (potentialX < POTENTIAL_MAX && potentialZ < POTENTIAL_MAX) {
		a = cX2 + cZ2;
		b = -2.0f * potentialX * cZ2 - 2.0f * potentialZ * cX2;
		c = cZ2 * potentialX * potentialX + cX2 * potentialZ * potentialZ - cX2 * cZ2;
	}
	else if (potentialX < POTENTIAL_MAX) {
		a = 1.0f;
		b = -2.0f * potentialX;
		c = potentialX * potentialX - cX2;
	}
	else {
		a = 1.0f;
		b = -2.0f * potentialZ;
		c = potentialZ * potentialZ - cZ2;
	}
	float qForm = quadFormLarger(a, b, c);
	return qForm;
}

void PotentialGrid::addCandidates(std::priority_queue<cell_potential> &candidatesPQ, int x, int z)
{
	float finiteDiff;

	int numNeighbors = 0; // debug

	// do potentials of N, S, E, W surrounding this coordinate
	// if the cell is in bounds, add to PQ
	// North
	if (m_potential->inBounds(x, z + 1) && m_known->getByIndex(x, z + 1) < (KNOWN_TAG + UNKNOWN_TAG)) {
		finiteDiff = finiteDifference(x, z + 1);
		cell_potential potential;
		potential.x = x;
		potential.z = z + 1;
		potential.potential = finiteDiff;
		candidatesPQ.push(potential);
		numNeighbors++;
	}
	// South
	if (m_potential->inBounds(x, z - 1) && m_known->getByIndex(x, z - 1) < (KNOWN_TAG + UNKNOWN_TAG)) {
		finiteDiff = finiteDifference(x, z - 1);
		cell_potential potential;
		potential.x = x;
		potential.z = z - 1;
		potential.potential = finiteDiff;
		candidatesPQ.push(potential);
		numNeighbors++;
	}
	// East
	if (m_potential->inBounds(x + 1, z) && m_known->getByIndex(x + 1, z) < (KNOWN_TAG + UNKNOWN_TAG)) {
		finiteDiff = finiteDifference(x + 1, z);
		cell_potential potential;
		potential.x = x + 1;
		potential.z = z;
		potential.potential = finiteDiff;
		candidatesPQ.push(potential);
		numNeighbors++;
	}
	// West
	if (m_potential->inBounds(x - 1, z) && m_known->getByIndex(x - 1, z) < (KNOWN_TAG + UNKNOWN_TAG)) {
		finiteDiff = finiteDifference(x - 1, z);
		cell_potential potential;
		potential.x = x - 1;
		potential.z = z;
		potential.potential = finiteDiff;
		candidatesPQ.push(potential);
		numNeighbors++;
	}
}

void PotentialGrid::splatGoal(Util::Point goalPosition)
{
	// http://stackoverflow.com/questions/9178083/priority-queue-for-user-defined-types
	std::priority_queue<cell_potential> candidatesPQ;
	int numCells = m_res_x * m_res_z;
	int numKnown = 0;

	int goal_idx_x;
	int goal_idx_z;
	m_potential->getIndicesForCoordinate(goalPosition.x, goalPosition.z, goal_idx_x, goal_idx_z);

	// begin by setting all potentials to infinity
	m_potential->clear(HUGE_VAL);
	m_potential->m_outOfBounds = HUGE_VAL;

	// mark all cells as unknown: neg is unknown
	m_known->clear(UNKNOWN_TAG);
	// set goal potential to 0.0
	m_potential->setByIndex(goal_idx_x, goal_idx_z, 0.0f);
	// mark goal as "known"
	m_known->setByIndex(goal_idx_x, goal_idx_z, KNOWN_TAG);
	numKnown++;

	// approximate finite difference for cells adjacent to "known." Add to the "candidate" list.
	addCandidates(candidatesPQ, goal_idx_x, goal_idx_z);

	while (numKnown < numCells && !candidatesPQ.empty()) {
		// pop candidate with lowest potential diff from pq. 
		cell_potential lowest = candidatesPQ.top();

		candidatesPQ.pop();
		if (std::isnan(lowest.potential)) { // shouldn't happen, might occur on boundaries
			continue;
		}
		// if candidate is already known, pop another.
		if (m_known->getByIndex(lowest.x, lowest.z) > (KNOWN_TAG + UNKNOWN_TAG)) continue;

		// update candidate's potential on the real grid
		m_potential->setByIndex(lowest.x, lowest.z, lowest.potential);
		// mark candidate as known, add neighbors to candidate set
		m_known->setByIndex(lowest.x, lowest.z, KNOWN_TAG);
		numKnown++;

		addCandidates(candidatesPQ, lowest.x, lowest.z);

		// done when all cells are known
	}
}

void PotentialGrid::computePotentialGradient()
{
	// just sample in the upwind direction
	float sample_1;
	float sample_2;
	// north
	for (int i = 0; i < m_res_x; i++) {
		for (int j = 0; j < m_res_z - 1; j++) {
			sample_1 = m_potential->getByIndex(i, j);
			sample_2 = m_potential->getByIndex(i, j + 1);
			m_dPotential_N->setByIndex(i, j, sample_2 - sample_1);
		}
	}
	// south
	for (int i = 0; i < m_res_x; i++) {
		for (int j = 1; j < m_res_z; j++) {
			sample_1 = m_potential->getByIndex(i, j);
			sample_2 = m_potential->getByIndex(i, j - 1);
			m_dPotential_S->setByIndex(i, j, sample_2 - sample_1);
		}
	}
	// east
	for (int i = 0; i < m_res_x - 1; i++) {
		for (int j = 0; j < m_res_z; j++) {
			sample_1 = m_potential->getByIndex(i, j);
			sample_2 = m_potential->getByIndex(i + 1, j);
			m_dPotential_E->setByIndex(i, j, sample_2 - sample_1);
		}
	}
	// west
	for (int i = 1; i < m_res_x; i++) {
		for (int j = 0; j < m_res_z; j++) {
			sample_1 = m_potential->getByIndex(i, j);
			sample_2 = m_potential->getByIndex(i - 1, j);
			m_dPotential_W->setByIndex(i, j, sample_2 - sample_1);
		}
	}
}

void PotentialGrid::renormalizeGradient() {
	float grad_max = m_dPotential_N->getMaxValAbs();
	grad_max = std::fmax(grad_max, m_dPotential_S->getMaxValAbs());
	grad_max = std::fmax(grad_max, m_dPotential_E->getMaxValAbs());
	grad_max = std::fmax(grad_max, m_dPotential_W->getMaxValAbs());

	float renorm = 1.0f / grad_max;
	m_dPotential_N->multiplyAll(renorm);
	m_dPotential_S->multiplyAll(renorm);
	m_dPotential_E->multiplyAll(renorm);
	m_dPotential_W->multiplyAll(renorm);
}

void PotentialGrid::computeSpeeds() {
	// to compute speed at a face, multiply grad by speed
	float dPotential;
	float speedSample;
	float newVal;
	for (int x = 0; x < m_res_x; x++) {
		for (int z = 0; z < m_res_z; z++) {
			// north
			dPotential = m_dPotential_N->getByIndex(x, z);
			speedSample = m_speeds_densities->m_speed_N->getByIndex(x, z);
			newVal = -dPotential * speedSample;
			m_velocity_N->setByIndex(x, z, newVal);

			// south
			dPotential = m_dPotential_S->getByIndex(x, z);
			speedSample = m_speeds_densities->m_speed_S->getByIndex(x, z);
			newVal = -dPotential * speedSample;
			m_velocity_S->setByIndex(x, z, newVal);

			// east
			dPotential = m_dPotential_E->getByIndex(x, z);
			speedSample = m_speeds_densities->m_speed_E->getByIndex(x, z);
			newVal = -dPotential * speedSample;
			m_velocity_E->setByIndex(x, z, newVal);

			// west
			dPotential = m_dPotential_W->getByIndex(x, z);
			speedSample = m_speeds_densities->m_speed_W->getByIndex(x, z);
			newVal = -dPotential * speedSample;
			m_velocity_W->setByIndex(x, z, newVal);
		}
	}
}

void PotentialGrid::update(Util::Point goalPosition)
{
	splatGoal(goalPosition);
	computePotentialGradient();
	//renormalizeGradient();
	computeSpeeds();
}

float lerp(float t, float valA, float valB) {
	return (1.0f - t)*valA + t*valB; // more precise, according to wikipedia
}

Util::Vector PotentialGrid::cellCenterVelocity(int x, int z)
{

	float v_N = m_velocity_N->getByIndex(x, z);
	float v_S = m_velocity_S->getByIndex(x, z);
	float v_E = m_velocity_E->getByIndex(x, z);
	float v_W = m_velocity_W->getByIndex(x, z);

	Util::Point center = m_potential->getCellCenter(x, z);


	float pos_z_N = center.z + m_potential->m_cell_size_z / 2.0f;
	float pos_z_S = center.z - m_potential->m_cell_size_z / 2.0f;
	float pos_x_E = center.x + m_potential->m_cell_size_x / 2.0f;
	float pos_x_W = center.x - m_potential->m_cell_size_x / 2.0f;

	// lerp in each direction to get the vector we want
	Util::Vector velocity;
	velocity.z = v_N - v_S;// lerp(0.5f, v_S, v_N);
	velocity.x = v_W - v_E;// lerp(0.5f, v_E, v_W);
	return velocity;
}


Util::Vector PotentialGrid::interpolateVelocity(Util::Point pos)
{
	/*
	float v_N = m_velocity_N->getByCoordinate(pos.x, pos.z);
	float v_S = m_velocity_S->getByCoordinate(pos.x, pos.z);
	float v_E = m_velocity_E->getByCoordinate(pos.x, pos.z);
	float v_W = m_velocity_W->getByCoordinate(pos.x, pos.z);

	Util::Point center = m_potential->getCellCenter(pos.x, pos.z);


	float pos_z_N = center.z + m_potential->m_cell_size_z / 2.0f;
	float pos_z_S = center.z - m_potential->m_cell_size_z / 2.0f;
	float pos_x_E = center.x + m_potential->m_cell_size_x / 2.0f;
	float pos_x_W = center.x - m_potential->m_cell_size_x / 2.0f;

	// lerp in each direction to get the vector we want
	Util::Vector velocity;
	float t_Z = (pos.z - pos_z_S) / m_potential->m_cell_size_z;
	velocity.z = lerp(t_Z, v_S, v_N);
	float t_X = (pos.x - pos_x_W) / m_potential->m_cell_size_x;
	velocity.x = lerp(t_X, v_E, v_W);
	return velocity; */

	// compute the cell center that is less than x, z
	Util::Point min = m_potential->getCellCenter(pos.x, pos.z);
	if (min.x > pos.x) min.x -= m_potential->m_cell_size_x;
	if (min.z > pos.z) min.z -= m_potential->m_cell_size_z;

	/****************
	max
	A-----B
	|     |
	|     |
	C-----D
	min

	*****************/
	int x, z;
	m_potential->getIndicesForCoordinate(min.x, min.z, x, z);

	Util::Vector c = cellCenterVelocity(x, z);
	Util::Vector d = cellCenterVelocity(x + 1, z);
	Util::Vector b = cellCenterVelocity(x + 1, z + 1);
	Util::Vector a = cellCenterVelocity(x, z + 1);

	float cd_x, cd_z;
	float ab_x, ab_z;

	float interp_x;
	float interp_z;

	// do lerps along one axis
	float t = (x - min.x) / m_potential->m_cell_size_x;
	cd_x = (1.0f - t) * c.x + d.x * t;
	ab_x = (1.0f - t) * a.x + b.x * t;
	cd_z = (1.0f - t) * c.z + d.z * t;
	ab_z = (1.0f - t) * a.z + b.z * t;

	// and then along the other!
	t = (z - min.z) / m_potential->m_cell_size_z;
	interp_x = (1.0f - t) * cd_x + t * ab_x;
	interp_z = (1.0f - t) * cd_z + t * ab_z;
	return Util::Vector(interp_x, 0.0f, interp_z);
}
