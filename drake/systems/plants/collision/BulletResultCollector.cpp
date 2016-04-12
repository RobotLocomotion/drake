#include "drake/systems/plants/collision/BulletResultCollector.h"

#include "drake/systems/plants/collision/DrakeCollision.h"

namespace DrakeCollision {
Eigen::Vector3d toVector3d(const btVector3& bt_vec) {
  Eigen::Vector3d vec;
  vec(0) = (double)bt_vec.getX();
  vec(1) = (double)bt_vec.getY();
  vec(2) = (double)bt_vec.getZ();
  return vec;
}

btScalar BulletResultCollector::addSingleResult(
    btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap,
    int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap,
    int partId1, int index1) {
  if ((colObj0Wrap->getCollisionObject()->getUserPointer() == nullptr) ||
      (colObj1Wrap->getCollisionObject()->getUserPointer() == nullptr)) {
    return false;
  }
  auto element0 = static_cast<Element*>(
      colObj0Wrap->getCollisionObject()->getUserPointer());
  auto element1 = static_cast<Element*>(
      colObj1Wrap->getCollisionObject()->getUserPointer());
  addSingleResult(element0->getId(), element1->getId(),
                  toVector3d(cp.getPositionWorldOnA()),
                  toVector3d(cp.getPositionWorldOnB()),
                  toVector3d(cp.m_normalWorldOnB), (double)cp.m_distance1);

  return 0;
}

btCollisionWorld::ContactResultCallback* BulletResultCollector::getBtPtr() {
  return static_cast<btCollisionWorld::ContactResultCallback*>(this);
}
}
