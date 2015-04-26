#include "trajectoryTestUtil.h"

using namespace std;

std::vector<double> generateRandomSegmentTimes(int num_segments, std::default_random_engine& generator) {
  vector<double> segment_times;
  uniform_real_distribution<double> uniform;
  double t0 = uniform(generator);
  segment_times.push_back(t0);
  for (int i = 0; i < num_segments; ++i) {
    double duration = uniform(generator);
    segment_times.push_back(segment_times[i] + duration);
  }
  return segment_times;
}
