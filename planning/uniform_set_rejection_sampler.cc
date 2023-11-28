#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/planning/uniform_set_sampler.h"

namespace drake {
namespace planning {
template <typename T>
UniformSetRejectionSampler<T>::UniformSetRejectionSampler(
    const T& set, const std::shared_pointer<ConvexSets> obstacles)
    : PointSamplerBase(), set_{set}, obstacles_{obstacles}, generator_{} {
  DRAKE_THROW_UNLESS(obstacles_.get() != nullptr);
}

template <typename T>
UniformSetRejectionSampler<T>::UniformSetRejectionSampler(
    const T& set, const std::shared_ptr<ConvexSets>& obstacles,
    const RandomGenerator& generator)
    : PointSamplerBase(),
      set_{set},
      obstacles_{obstacles},
      generator_{generator} {
  DRAKE_THROW_UNLESS(obstacles_.get() != nullptr);
}

template <typename T>
Eigen::MatrixXd UniformSetRejectionSampler<T>::DoSamplePoints(int num_points) {
  Eigen::MatrixXd ret(set_.ambient_dimension(), num_points);
  int points_added{0};
  while (points_added < num_points) {
    const Eigen::MatrixXd sample = set_.UniformSample(&generator_);
    bool do_add{true};
    for (const auto& obstacle : obstacles_) {
      // TODO(Alexandre.Amice) ensure that we use the HPolyhedron hit and run
      // sampler here.
      do_add = !obstacle.PointInSet(sample);
      if (!do_add) {
        break;
      }
    }
    if (do_add) {
      ret.col(points_added) = sample;
      ++points_added;
    }
  }
  return ret;
}

// Explicit instantiation for HPolyhedron and Hyperrectangle.
template class UniformSetRejectionSampler<geometry::optimization::HPolyhedron>;
template class UniformSetRejectionSampler<
    geometry::optimization::Hyperrectangle>;

}  // namespace planning
}  // namespace drake