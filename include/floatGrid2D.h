#ifndef __FLOAT_GRID_2D__
#define __FLOAT_GRID_2D__

#include "SteerLib.h"

class float_grid_2D{
public:
	float_grid_2D(int res_x, int res_z, Util::Point min, Util::Point max);
	~float_grid_2D();
	float getByIndex(int x, int z);
	bool setByIndex(int x, int z, float val);
	bool addByIndex(int x, int z, float val);
	void clear(float val);

	void getIndicesForCoordinate(float x, float z, int &x_cell, int &z_cell);

	// grid is m_res_x by m_res_z between m_min and m_max
	float getByCoordinate(float x, float z);
	bool setByCoordinate(float x, float z, float val);

	Util::Point getCornerOfIndex(int x_cell, int z_cell);
	Util::Point getCellCenter(float x, float z);
	float getMaxVal(int &x, int &z);
	float getMaxVal();
	float getMaxValAbs();
	float getMinVal();
	void multiplyAll(float m);

	bool inBounds(int x, int z);
	bool finBounds(float x, float z);

	float bilinearInterp(float x, float z);

	std::vector<float> m_values;
	int m_res_x;
	int m_res_z;
	float m_cell_size_x;
	float m_cell_size_z;
	Util::Point m_min;
	Util::Point m_max;
	float m_outOfBounds;
};

#endif