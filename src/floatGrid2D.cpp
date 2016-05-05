#include "floatGrid2D.h"

float_grid_2D::float_grid_2D(int res_x, int res_z, Util::Point min, Util::Point max)
{
	m_res_x = res_x;
	m_res_z = res_z;
	m_min = min;
	m_max = max;
	m_values.resize(res_x * res_z);
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
	if (x < 0 || x >= m_res_x || z < 0 || z >= m_res_z) return -HUGE_VAL;
	return m_values.at(x + z * m_res_x);
}

bool float_grid_2D::setByIndex(int x, int z, float val)
{
	if (x < 0 || x >= m_res_x || z < 0 || z >= m_res_z) return false;
	m_values.at(x + z * m_res_x) = val;
	return true;
}

bool float_grid_2D::addByIndex(int x, int z, float val)
{
	if (x < 0 || x >= m_res_x || z < 0 || z >= m_res_z) return false;
	m_values.at(x + z * m_res_x) += val;
	return true;
}

void float_grid_2D::clear(float val)
{
	for (int i = 0; i < m_res_x * m_res_z; i++) {
		m_values[i] = val;
	}
}

void float_grid_2D::getIndicesForCoordinate(float x, float z, int &x_cell, int &z_cell)
{
	float coord_localized_x = x - m_min.x;
	float coord_localized_z = z - m_min.z;

	x_cell = int(coord_localized_x / m_cell_size_x);
	z_cell = int(coord_localized_z / m_cell_size_z);
}

float float_grid_2D::getByCoordinate(float x, float z)
{
	int x_idx;
	int z_idx;
	getIndicesForCoordinate(x, z, x_idx, z_idx);

	return getByIndex(x_idx, z_idx);
}

bool float_grid_2D::setByCoordinate(float x, float z, float val)
{
	int x_idx;
	int z_idx;
	getIndicesForCoordinate(x, z, x_idx, z_idx);

	return setByIndex(x_idx, z_idx, val);
}

Util::Point float_grid_2D::getCornerOfIndex(int x_cell, int z_cell)
{
	Util::Point corner;
	corner.x = x_cell * m_cell_size_x + m_min.x;
	corner.z = z_cell * m_cell_size_z + m_min.z;
	return corner;
}

Util::Point float_grid_2D::getCellCenter(float x, float z)
{
	// return the coordinates of the cell center that this cell is in

	int x_idx;
	int z_idx;
	getIndicesForCoordinate(x, z, x_idx, z_idx);

	// now get coordinates of cell center: coordinates of corner + half cell width
	Util::Point center;
	center.x = x_idx * m_cell_size_x + m_cell_size_x * 0.5f + m_min.x;
	center.z = z_idx * m_cell_size_z + m_cell_size_z * 0.5f + m_min.z;
	return center;
}

float float_grid_2D::getMaxVal(int &x, int &z) {
	float maxVal = -HUGE_VAL;
	float candidate;
	for (int ix = 0; ix < m_res_x; ix++) {
		for (int iz = 0; iz < m_res_z; iz++) {
			candidate = getByIndex(ix, iz);
			if (candidate > maxVal) {
				maxVal = candidate;
				x = ix;
				z = iz;
			}
		}
	}
	return maxVal;
}

bool float_grid_2D::inBounds(int x, int z)
{
	return (x >= 0 && x < m_res_x && z >= 0 && z < m_res_z);
}

bool float_grid_2D::finBounds(float x, float z)
{
	return (x >= m_min.x && x < m_max.x && z >= m_min.z && z < m_max.z);
}