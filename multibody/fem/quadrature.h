#pragma once

#include <array>
#include <utility>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* A base class for quadratures that facilitates numerical integrations in FEM.
 This class provides the common methods and the data for all the quadrature
 rules with no heap allocation or virtual methods. The specification of
 particular quadrature rules will be provided in the derived classes.
 @tparam natural_dimension_at_compile_time  Dimension of the domain of
                                            integration.
 @tparam num_locations_at_compile_time      Number of quadrature locations. */
template <int natural_dimension_at_compile_time,
          int num_quadrature_points_at_compile_time>
class Quadrature {
 public:
  /* The dimension of the parent domain. */
  static constexpr int natural_dimension = natural_dimension_at_compile_time;

  /* The number of quadrature locations. */
  static constexpr int num_quadrature_points =
      num_quadrature_points_at_compile_time;

  static_assert(1 <= natural_dimension && natural_dimension <= 3,
                "Only 1, 2 and 3 dimensional shapes are supported.");
  static_assert(1 <= num_quadrature_points,
                "There has to be at least one quadrature point.");

  using VectorD = Eigen::Matrix<double, natural_dimension, 1>;
  using LocationsType = std::array<VectorD, num_quadrature_points>;
  using WeightsType = std::array<double, num_quadrature_points>;

  /* The position in parent domain coordinates of all quadrature points. */
  const LocationsType& get_points() const { return points_; }

  /* The weight of all quadrature points. */
  const WeightsType& get_weights() const { return weights_; }

  /* The position in parent domain coordinates of the q-th quadrature point. */
  const VectorD& get_point(int q) const {
    DRAKE_ASSERT(0 <= q && q < num_quadrature_points);
    return points_[q];
  }

  /* The weight of the q-th quadrature point. */
  double get_weight(int q) const {
    DRAKE_ASSERT(0 <= q && q < num_quadrature_points);
    return weights_[q];
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Quadrature);

  /* Constructs a Quadrature object given the quadrature locations and the
   weights of the quadrature points. */
  explicit Quadrature(
      std::pair<LocationsType, WeightsType>&& points_and_weights)
      : points_(std::move(points_and_weights.first)),
        weights_(std::move(points_and_weights.second)) {}

 private:
  LocationsType points_;
  WeightsType weights_;
};

template <class ObjectType>
struct is_quadrature {
  static constexpr bool value =
      std::is_base_of_v<Quadrature<ObjectType::natural_dimension,
                                   ObjectType::num_quadrature_points>,
                        ObjectType>;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
