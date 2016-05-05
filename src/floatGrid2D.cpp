#include "floatGrid2D.h"

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

void float_grid_2D::getCornerOfIndex(int x_cell, int z_cell, float &x, float &z)
{
	x = x_cell * m_cell_size_x + m_min.x;
	z = z_cell * m_cell_size_z + m_min.z;
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