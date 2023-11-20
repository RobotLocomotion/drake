#pragma once

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/approximate_convex_cover_builder_base.h"

namespace drake {
namespace planning {

class UniformPolytopeSampler : public PointSamplerBase {
 public:
  UniformPolytopeSampler(const HPolyhedron& polytope);

  UniformPolytopeSampler(const HPolyhedron& polytope,
                         const RandomGenerator& generator);

  UniformPolytopeSampler(const Eigen::Ref<const Eigen::MatrixXd>& A,
                         const Eigen::Ref<const Eigen::VectorXd>& b);

  UniformPolytopeSampler(const Eigen::Ref<const Eigen::MatrixXd>& A,
                         const Eigen::Ref<const Eigen::VectorXd>& b,
                         const RandomGenerator& generator);

  Eigen::MatrixXd SamplePoints(int num_points);

 private:
  const geometry::optimization::HPolyhedron polytope_;
  RandomGenerator generator_;
};
}  // namespace planning
}  // namespace drake