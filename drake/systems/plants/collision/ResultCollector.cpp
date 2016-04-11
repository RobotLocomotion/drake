#include <iostream>

#include "drake/systems/plants/collision/DrakeCollision.h"
#include "ResultCollector.h"

using namespace std;
using namespace Eigen;

namespace DrakeCollision {
void ResultCollector::addPointPairResult(const PointPair& result) {
  pts.push_back(result);
}

PointPair ResultCollector::minDistPoint() {
  return *min_element(pts.begin(), pts.end());
}
}
