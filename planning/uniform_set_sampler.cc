#include "drake/planning/uniform_set_sampler.h"

namespace drake {
namespace planning {
using geometry::optimization::Hyperrectangle;
using geometry::optimization::HPolyhedron;

UniformSetSampler<HPolyhedron>::UniformSetSampler(const HPolyhedron& set)
    : set_(set), generator_() {}

//UniformSetSampler<>::UniformSetSampler(const HPolyhedron& set,
//                                        const RandomGenerator& generator)
//    : set_(set), generator_(generator) {}

template <typename T>
Eigen::MatrixXd UniformSetSampler<T>::SamplePoints(int num_points) {
  Eigen::MatrixXd ret(set_.ambient_dimension(), num_points);
  for (int i = 0; i < num_points; ++i) {
    ret.col(i) = set_.UniformSample(&generator_);
  }
  return ret;
}
}  // namespace planning
}  // namespace drake