#ifndef DRAKE_SYSTEMS_PLANTS_COLLISION_RESULTCOLLECTOR_H_
#define DRAKE_SYSTEMS_PLANTS_COLLISION_RESULTCOLLECTOR_H_

#include "drake/systems/plants/collision/DrakeCollision.h"

namespace DrakeCollision {
class ResultCollector {
 public:
  virtual ~ResultCollector() {}

  virtual void addPointPairResult(const PointPair& result);

  inline void addSingleResult(const ElementId idA, const ElementId idB,
                              const Eigen::Vector3d& ptA,
                              const Eigen::Vector3d& ptB,
                              const Eigen::Vector3d& normal, double distance) {
    addPointPairResult(PointPair(idA, idB, ptA, ptB, normal, distance));
  }

  std::vector<PointPair> getResults() const { return pts; }

  PointPair minDistPoint();

  std::vector<PointPair> pts;
};

typedef std::shared_ptr<ResultCollector> ResultCollShPtr;
}

#endif  // DRAKE_SYSTEMS_PLANTS_COLLISION_RESULTCOLLECTOR_H_
