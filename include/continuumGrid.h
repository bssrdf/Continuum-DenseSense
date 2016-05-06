#ifndef __CONTINUUM_GRID__
#define __CONTINUUM_GRID__
#include "floatGrid2D.h"

#define DENSITY_FALLOFF 0.9f
#define DENSITY_MIN 0.00001f // for preventing divide by zero errors
#define COST_SMOOTH 0.00001f

#define SPEED_WEIGHT 0.7f
#define TIME_WEIGHT 0.2f
#define DISCOMFORT_WEIGHT 0.1f

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
	float_grid_2D *m_discomfort;
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
	void splatAgent(Util::Point agentPosition, Util::Vector agentVelocity); // section 4.1
	void splatObstacle(Util::Point obstaclePosition);
	
	void normalizeVelocitiesByDensity(); // equation 7
	void computeSpeedFields(); // equation 9, 10
	void computeCostFields();
};

#endif