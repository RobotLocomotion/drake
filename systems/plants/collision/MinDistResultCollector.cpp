#include "MinDistResultCollector.h"

using namespace std;

namespace DrakeCollision
{
  void MinDistResultCollector::addPointPairResult(const PointPair& result)
  {
    if (pts.size() > 0) { 
      if (result >= pts[0]) {
        return;
      } else {
        pts.pop_back();
      }
    } 
    ResultCollector::addPointPairResult(result);
  };
}
