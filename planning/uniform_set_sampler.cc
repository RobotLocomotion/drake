#include "drake/planning/uniform_set_sampler.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"

namespace drake {
namespace planning {
template <typename T>
UniformSetSampler<T>::UniformSetSampler(const T& set)
    : set_{set}, generator_{} {}

template <typename T>
UniformSetSampler<T>::UniformSetSampler(const T& set,
                                        const RandomGenerator& generator)
    : set_{set}, generator_{generator} {}

template <typename T>
Eigen::MatrixXd UniformSetSampler<T>::DoSamplePoints(int num_points) {
  Eigen::MatrixXd ret(set_.ambient_dimension(), num_points);
  for (int i = 0; i < num_points; ++i) {
    ret.col(i) = set_.UniformSample(&generator_);
  }
  return ret;
}

// Explicit instantiation for HPolyhedron and Hyperrectangle.
template class UniformSetSampler<geometry::optimization::HPolyhedron>;
template class UniformSetSampler<geometry::optimization::Hyperrectangle>;

}  // namespace planning
}  // namespace drake