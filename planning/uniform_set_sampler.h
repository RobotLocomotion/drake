#pragma once

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/planning/approximate_convex_cover_builder_base.h"

namespace drake {
namespace planning {
using geometry::optimization::Hyperrectangle;
using geometry::optimization::HPolyhedron;

//template <typename T,
//          typename = std::enable_if_t<std::is_same<T, Hyperrectangle>::value ||
//                                      std::is_same<T, HPolyhedron>::value>>
template <typename T>
class UniformSetSampler: public PointSamplerBase {
 public:
  UniformSetSampler(const T& set);
  UniformSetSampler(const T& set, const RandomGenerator& generator);

  Eigen::MatrixXd SamplePoints(int num_points);

 private:
  const T set_;
  RandomGenerator generator_;
};

template<>
UniformSetSampler<HPolyhedron>A
}  // namespace planning
}  // namespace drake