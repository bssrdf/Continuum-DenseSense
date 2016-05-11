#ifndef __POTENTIAL_GRID__
#define __POTENTIAL_GRID__
#include "continuumGrid.h"
#include "floatGrid2D.h"

#define UNKNOWN_TAG -1.0f
#define KNOWN_TAG 1.0f
#define POTENTIAL_MAX 1000000000000000.0f // for quadform in finite difference

struct cell_potential{
	// cell coordinates of this potential
	int x;
	int z;
	// said potential
	float potential;
	// comparator, for PQ
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

	float_grid_2D *m_velocity_X; // final velocity X
	float_grid_2D *m_velocity_Z; // final velocity Z

	int m_res_x;
	int m_res_z;

	ContinuumGrid *m_speeds_densities;

	PotentialGrid(int res_x, int res_z, Util::Point min, Util::Point max, ContinuumGrid *speeds_densities);
	~PotentialGrid();

	void update(Util::Point goalPosition);

	Util::Vector cellCenterVelocity(int x, int z);
	Util::Vector interpolateVelocity(Util::Point pos);

private:
	float finiteDifference(int x, int z);
	void addCandidates(std::priority_queue<cell_potential> &candidatesPQ, int x, int z);
	void splatGoal(Util::Point goalPosition); // handle potential changes as a result of adding a goal
	
	// called by update: no need to call this on its own
	void computePotentialGradient();
	void renormalizeGradient();
	void computeSpeeds();
};

#endif