#include "drake/systems/plants/collision/ResultCollector.h"

#include "drake/systems/plants/collision/DrakeCollision.h"

namespace DrakeCollision {
void ResultCollector::addPointPairResult(const PointPair& result) {
  pts.push_back(result);
}

PointPair ResultCollector::minDistPoint() {
  return *std::min_element(pts.begin(), pts.end());
}
}
