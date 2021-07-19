#pragma once

#include <benchmark/benchmark.h>

namespace drake {
namespace tools {
namespace performance {

/** Adds "min" and "max" statistics to the fixture's reports. */
void AddMinMaxStatistics(benchmark::Fixture* fixture);

}  // namespace performance
}  // namespace tools
}  // namespace drake
