#include "drake/tools/performance/fixture_common.h"

#include <algorithm>
#include <vector>

namespace drake {
namespace tools {
namespace performance {

void AddMinMaxStatistics(benchmark::Fixture* fixture) {
  fixture->ComputeStatistics("min", [](const std::vector<double>& v) -> double {
    return *(std::min_element(std::begin(v), std::end(v)));
  });
  fixture->ComputeStatistics("max", [](const std::vector<double>& v) -> double {
    return *(std::max_element(std::begin(v), std::end(v)));
  });
}

}  // namespace performance
}  // namespace tools
}  // namespace drake
