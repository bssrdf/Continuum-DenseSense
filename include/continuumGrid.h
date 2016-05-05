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
	/********************************************************
	Grid for global values

	At each cell center:
	- agent density
	- discomfort density
	- average velocity x
	- average velocity z
	At each cell face:
	- speed field to adjacent cell - anisotropic
	- cost function to adjacent cell - anisotropic
	*********************************************************/
public:
	// center values
	float_grid_2D *m_density;
	float_grid_2D *m_avg_vel_x; // sum(p_i * vx_i) / p
	float_grid_2D *m_avg_vel_z; // sum(p_i * vz_i) / p
	
	// face values
	float_grid_2D *m_speed_N;
	float_grid_2D *m_speed_S;
	float_grid_2D *m_speed_E;
	float_grid_2D *m_speed_W;

	float_grid_2D *m_cost_N;
	float_grid_2D *m_cost_S;
	float_grid_2D *m_cost_E;
	float_grid_2D *m_cost_W;

	int m_res_x;
	int m_res_z;
	Util::Point m_min;
	Util::Point m_max;

	float m_max_density;

	ContinuumGrid(int res_x, int res_z, Util::Point min, Util::Point max);
	~ContinuumGrid();

	void resetSplats();
	void splatAgent(Util::Point agentPosition, Util::Vector agentVelocity);
	void splatObstacle(Util::Point obstaclePosition);
	
	void normalizeVelocitiesByDensity();
	void computeSpeedFields();
	void computeCostFields();
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
	/***************************************************************
	Grid for goal-specific values.
	
	At each cell center:
	-potential
	
	At each cell face:
	-change in potential
	-velocity from change in potential + speed field on global grid
	****************************************************************/


public:
	// center values
	float_grid_2D *m_potential;
	float_grid_2D *m_known; // for finite difference fast flood fill

	// face values
	float_grid_2D *m_dPotential_N;
	float_grid_2D *m_dPotential_S;
	float_grid_2D *m_dPotential_E;
	float_grid_2D *m_dPotential_W;

	float_grid_2D *m_velocity_N;
	float_grid_2D *m_velocity_S;
	float_grid_2D *m_velocity_E;
	float_grid_2D *m_velocity_W;

	int m_res_x;
	int m_res_z;

	ContinuumGrid *m_speeds_densities;

	PotentialGrid(int res_x, int res_z, Util::Point min, Util::Point max, ContinuumGrid *speeds_densities);
	~PotentialGrid();

	void update(Util::Point goalPosition);

private:
	//float finiteDifference(int x, int z);
	//void computeSpeedField();
	//void computeUnitCosts();
	//void addCandidates(std::priority_queue<cell_potential> &candidatesPQ, int x, int z);
	//void splatGoal(Util::Point goalPosition); // handle potential changes as a result of adding a goal
	//
	//void computePotentialDeltas(); // called by splatGoal: no need to call this on its own
};


#endif