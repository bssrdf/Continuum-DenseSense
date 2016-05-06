#include "continuumGrid.h"

ContinuumGrid::ContinuumGrid(int res_x, int res_z, Util::Point min, Util::Point max)
{
	m_density = new float_grid_2D(res_x, res_z, min, max);
	m_discomfort = new float_grid_2D(res_x, res_z, min, max);

	m_avg_vel_x = new float_grid_2D(res_x, res_z, min, max);
	m_avg_vel_z = new float_grid_2D(res_x, res_z, min, max);
	m_avg_vel_x->m_outOfBounds = 0.0f;
	m_avg_vel_z->m_outOfBounds = 0.0f;

	m_speed_N = new float_grid_2D(res_x, res_z, min, max);
	m_speed_S = new float_grid_2D(res_x, res_z, min, max);
	m_speed_E = new float_grid_2D(res_x, res_z, min, max);
	m_speed_W = new float_grid_2D(res_x, res_z, min, max);

	m_cost_N = new float_grid_2D(res_x, res_z, min, max);
	m_cost_S = new float_grid_2D(res_x, res_z, min, max);
	m_cost_E = new float_grid_2D(res_x, res_z, min, max);
	m_cost_W = new float_grid_2D(res_x, res_z, min, max);

	m_res_x = res_x;
	m_res_z = res_z;
	m_min = min;
	m_max = max;
}

ContinuumGrid::~ContinuumGrid()
{
	delete m_density;
	delete m_discomfort;
	delete m_avg_vel_x;
	delete m_avg_vel_z;

	delete m_speed_N;
	delete m_speed_S;
	delete m_speed_E;
	delete m_speed_W;

	delete m_cost_N;
	delete m_cost_S;
	delete m_cost_E;
	delete m_cost_W;
}

void ContinuumGrid::resetSplats()
{
	m_density->clear(DENSITY_MIN); // to prevent div by zero errors
	m_avg_vel_x->clear(0.0f);
	m_avg_vel_z->clear(0.0f);

	// for now
	m_discomfort->clear(0.0f);
}

void ContinuumGrid::splatAgent(Util::Point agentPosition, Util::Vector agentVelocity)
{
	// section 4.1

	// get the position of the closest cell with center coordinates < agentPosition
	Util::Point center = m_density->getCellCenter(agentPosition.x, agentPosition.z);

	// it's ok if this ends up being out of bounds, will be handled by splatting below.

	if (center.x > agentPosition.x) center.x -= m_density->m_cell_size_x;
	if (center.z > agentPosition.z) center.z -= m_density->m_cell_size_z;

	int x_cell_idx;
	int z_cell_idx;
	m_density->getIndicesForCoordinate(center.x, center.z, x_cell_idx, z_cell_idx);

	// compute contribs to this cell and each cell w/larger coord
	float cell_width = m_density->m_cell_size_x;
	float cell_height = m_density->m_cell_size_z;

	// compute coords relative to center. normalize to cell scale.
	float dx = (agentPosition.x - center.x) / cell_width;
	float dz = (agentPosition.z - center.z) / cell_height;

	float pA = pow(min(1.0f - dx, 1.0f - dz), DENSITY_FALLOFF); // this cell
	float pB = pow(min(dx, 1.0f - dz), DENSITY_FALLOFF); // right
	float pC = pow(min(dx, dz), DENSITY_FALLOFF); // up-right
	float pD = pow(min(1.0f - dx, dz), DENSITY_FALLOFF); // up

	// add contributions to each cell with coordinates larger
	m_density->addByIndex(x_cell_idx, z_cell_idx, pA);
	m_density->addByIndex(x_cell_idx + 1, z_cell_idx, pB);
	m_density->addByIndex(x_cell_idx + 1, z_cell_idx + 1, pC);
	m_density->addByIndex(x_cell_idx, z_cell_idx + 1, pD);

	// compute avg velocity contributions - equation 7
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
	// equation 7
	// divide each cell's velocity sums by the sum of density here
	float sum_v_x;
	float sum_v_z;
	float sum_p;
	for (int x = 0; x < m_res_x; x++) {
		for (int z = 0; z < m_res_z; z++) {
			sum_p = m_density->getByIndex(x, z);

			sum_v_x = m_avg_vel_x->getByIndex(x, z);
			m_avg_vel_x->setByIndex(x, z, sum_v_x / sum_p);

			sum_v_z = m_avg_vel_z->getByIndex(x, z);
			m_avg_vel_z->setByIndex(x, z, sum_v_z / sum_p);
		}
	}
}

void ContinuumGrid::computeSpeedFields() {
	// for every face in every cell, compute flow speed into the next cell
	
	// EQ9: f_v(x, theta) = v(x + rn_theta) dot n_theta

	// since we're doing this on a mac grid, this basically means:
	// - sample the velocity component in this direction in the next grid over
	
	// EQ10: f(x, theta) = ((p(x + rn_theta) - pmin) / (pmax - pmin)) * f_v
	// - sample the density in the next cell over

	float p_max = 0.8f;
	float p_min = 0.3f;

	float f_v; // speed field value from sampling next cell over
	float p_n; // pressure in next cell over
	float f; // speed field computation
	float f_t = 5.0f; // in absence of other stuff, should always be happy moving away from your cell

	for (int x = 0; x < m_res_x; x++) {
		for (int z = 0; z < m_res_z; z++) {
			// don't need to worry about boundary conditions: out of bounds returns 0.0f

			// North, aka z+
			f_v = m_avg_vel_z->getByIndex(x, z + 1);
			p_n = m_density->getByIndex(  x, z + 1);
			f = f_t + (p_n - p_min) / (p_max - p_min) * (f_v - f_t);
			if (p_n > p_max) 
				f = f_v;
			if (p_n < p_min) 
				f = f_t;
			m_speed_N->setByIndex(x, z, f);

			// South, aka z-
			f_v = m_avg_vel_z->getByIndex(x, z - 1) * -1.0f; // dot product with [0, -1]
			p_n = m_density->getByIndex(  x, z - 1);
			f = f_t + (p_n - p_min) / (p_max - p_min) * (f_v - f_t);
			if (p_n > p_max)
				f = f_v * -1.0f;
			if (p_n < p_min) 
				f = f_t * -1.0f;
			m_speed_S->setByIndex(x, z, f);

			// East, aka x+
			f_v = m_avg_vel_x->getByIndex(x + 1, z);
			p_n = m_density->getByIndex(  x + 1, z);
			f = f_t + (p_n - p_min) / (p_max - p_min) * (f_v - f_t);
			if (p_n > p_max) 
				f = f_v;
			if (p_n < p_min) 
				f = f_t;
			m_speed_E->setByIndex(x, z, f);

			// West, aka x-
			f_v = m_avg_vel_x->getByIndex(x - 1, z) * -1.0f; // dot product with [-1, 0]
			p_n = m_density->getByIndex(  x - 1, z);
			f = f_t + (p_n - p_min) / (p_max - p_min) * (f_v - f_t);
			if (p_n > p_max) 
				f = f_v * -1.0f;
			if (p_n < p_min) 
				f = f_t * -1.0f;
			m_speed_W->setByIndex(x, z, f);
		}
	}
}

void ContinuumGrid::computeCostFields() {
	// equation 4. also anisotropic.
	// C = (alpha * flowSpeed + beta + delta * discomfort) / flowSpeed
	// essentially, cost of moving from this cell to the next
	// it's worse for you if you're moving into a cell that has velocity flowing in the opposite dir
	// should this be negative or positive? paper equations indicate negative, use a dot product
	
	float f; // speed field value -> anisotropic
	float g; // discomfort
	float c; // cost -> anisotropic

	for (int x = 0; x < m_res_x; x++) {
		for (int z = 0; z < m_res_z; z++) {
			// don't need to worry about boundary conditions: out of bounds returns 0.0f

			g = m_discomfort->getByIndex(x, z);

			// North, aka z+
			f = m_speed_N->getByIndex(x, z);// +COST_SMOOTH;
			c = (SPEED_WEIGHT * f + TIME_WEIGHT + DISCOMFORT_WEIGHT * g) / f;
			m_cost_N->setByIndex(x, z, c);

			// South, aka z-
			f = m_speed_S->getByIndex(x, z);// -COST_SMOOTH;
			c = (SPEED_WEIGHT * f + TIME_WEIGHT + DISCOMFORT_WEIGHT * g) / f;
			m_cost_S->setByIndex(x, z, c);

			// East, aka x+
			f = m_speed_E->getByIndex(x, z);// +COST_SMOOTH;
			c = (SPEED_WEIGHT * f + TIME_WEIGHT + DISCOMFORT_WEIGHT * g) / f;
			m_cost_E->setByIndex(x, z, c);

			// West, aka x-
			f = m_speed_W->getByIndex(x, z);// -COST_SMOOTH;
			c = (SPEED_WEIGHT * f + TIME_WEIGHT + DISCOMFORT_WEIGHT * g) / f;
			m_cost_W->setByIndex(x, z, c);
		}
	}
}