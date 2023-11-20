#include "drake/planning/uniform_polytope_sampler.h"

namespace drake {
namespace planning {

UniformPolytopeSampler::UniformPolytopeSampler(const HPolyhedron& polytope)
    : polytope_(polytope), generator_() {}

UniformPolytopeSampler::UniformPolytopeSampler(const HPolyhedron& polytope,
                                               const RandomGenerator& generator)
    : polytope_(polytope), generator_(generator) {}

UniformPolytopeSampler::UniformPolytopeSampler(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b)
    : polytope_(A, b), generator_(){};

UniformPolytopeSampler::UniformPolytopeSampler(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const RandomGenerator& generator)
    : polytope_(A, b), generator_(generator){};

Eigen::MatrixXd UniformPolytopeSampler::SamplePoints(int num_points) {
  Eigen::MatrixXd ret(bounding_box_.ambient_dimension(), num_points);
  for (int i = 0; i < num_points; ++i) {
    ret.col(i) = polytope_.UniformSample(&generator_);
  }
  return ret;
}
}  // namespace planning
}  // namespace drake