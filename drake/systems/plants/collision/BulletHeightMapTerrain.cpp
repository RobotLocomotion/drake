#include <iostream>
#include <fstream>

#include "BulletHeightMapTerrain.h"

using namespace std;
using namespace Eigen;

namespace DrakeCollision {

void GatherHeightMapAsGridCallBack::processTriangle(btVector3* triangle,
                                                    int i, int j) {
  int n = m_grid_points.size();
  m_connectivities.push_back(Vector3i(n + 1, n + 2, n + 3));
  m_grid_points.push_back(triangle[0]);
  m_grid_points.push_back(triangle[1]);
  m_grid_points.push_back(triangle[2]);
}

}  // namespace DrakeCollision
