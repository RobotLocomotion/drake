#ifndef DRAKE_SYSTEMS_PLANTS_COLLISION_BULLETHEIGHTMAPTERRAIN_H_
#define DRAKE_SYSTEMS_PLANTS_COLLISION_BULLETHEIGHTMAPTERRAIN_H_

#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <Eigen/StdVector>

#include "../shapes/HeightMapTerrain.h"

namespace DrakeCollision {

struct btTriangle {
  btVector3 m_vertex0;
  btVector3 m_vertex1;
  btVector3 m_vertex2;
  int m_partId;
  int m_triangleIndex;
};

class GatherHeightMapAsGridCallBack : public btTriangleCallback {
  bool first_triangle;

  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> >
      m_connectivities;
  btAlignedObjectArray<btVector3> m_grid_points;

 public:
  GatherHeightMapAsGridCallBack() : first_triangle(true) {}

  virtual void processTriangle(btVector3* triangle, int i, int j);

  int numTriangles() const { return int(m_connectivities.size()); }

  int numPoints() const { return int(m_grid_points.size()); }

  const btVector3& getPoint(int i) const { return m_grid_points[i]; }

  const Eigen::Vector3i& getTriangle(int i) const {
    return m_connectivities[i];
  }
};

}  // namespace DrakeCollision

#endif  // DRAKE_SYSTEMS_PLANTS_COLLISION_BULLETHEIGHTMAPTERRAIN_H_
