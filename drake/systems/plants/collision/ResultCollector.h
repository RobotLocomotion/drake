#pragma once

#include "drake/systems/plants/collision/DrakeCollision.h"

namespace DrakeCollision {
class ResultCollector {
 public:
  virtual ~ResultCollector() {}

  virtual void addPointPairResult(const PointPair& result);

  void addSingleResult(const Element* elementA, const Element* elementB,
                       const Eigen::Vector3d& ptA,
                       const Eigen::Vector3d& ptB,
                       const Eigen::Vector3d& normal,
                       const double distance) {
    pts.emplace_back(elementA, elementB, ptA, ptB, normal, distance);
  }

  std::vector<PointPair> getResults() const { return pts; }

  PointPair minDistPoint() const;

  std::vector<PointPair> pts;
};

typedef std::shared_ptr<ResultCollector> ResultCollShPtr;
}
