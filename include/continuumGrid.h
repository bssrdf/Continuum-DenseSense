#ifndef __CONTINUUM_GRID__
#define __CONTINUUM_GRID__
#include "floatGrid2D.h"

#define DENSITY_FALLOFF 0.9f
#define TIME_WEIGHT 0.7f
#define SPEED_WEIGHT 0.9f
#define DENSITY_MIN 0.001f
#define POTENTIAL_MAX 1000.0f

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
	float_grid_2D *m_known; // for finite difference fast flood fill
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


#endif