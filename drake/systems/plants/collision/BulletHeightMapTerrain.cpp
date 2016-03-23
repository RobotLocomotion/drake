#include <iostream>
#include <fstream>

#include "BulletHeightMapTerrain.h"

using namespace std;
using namespace Eigen;

namespace DrakeCollision {

	void GatherHeightMapAsGridCallBack::processTriangle(btVector3* triangle, int i, int j){		
		int n = m_grid_points.size();

		m_connectivities.push_back(Vector3i(n+1,n+2,n+3));		

		cout << "i,j: " << i << " " << j << ": " << endl;
		cout << "triangle[0]:" << triangle[0][0] << " " << triangle[0][1] << " " << triangle[0][2] << endl;
		cout << "triangle[1]:" << triangle[1][0] << " " << triangle[1][1] << " " << triangle[1][2] << endl;
		cout << "triangle[2]:" << triangle[2][0] << " " << triangle[2][1] << " " << triangle[2][2] << endl;

		m_grid_points.push_back(triangle[0]);
		m_grid_points.push_back(triangle[1]);
		m_grid_points.push_back(triangle[2]);		
	}

}//namespace DrakeCollision