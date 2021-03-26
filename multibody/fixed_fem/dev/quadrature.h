#pragma once

#include <array>
#include <string>
#include <utility>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
// TODO(xuchenhan-tri): Consider allowing AutoDiffScalar if it simplifies the
// syntax in the AutoDiff case.
/** A base class for quadratures that facilitates numerical integrations in FEM.
 This class provides the common interface and the data for all the quadrature
 rules with no heap allocation or virtual methods. The specification of
 particular quadrature rules will be provided in the derived classes.
 @tparam NaturalDimension dimension of the domain of integration.
 @tparam NumLocations number of quadrature locations. */
template <int NaturalDimension, int NumLocations>
class Quadrature {
 public:
  static_assert(1 <= NaturalDimension && NaturalDimension <= 3,
                "Only 1, 2 and 3 dimensional shapes are supported.");
  static_assert(1 <= NumLocations,
                "There has to be at least one quadrature point.");

  using VectorD = Eigen::Matrix<double, NaturalDimension, 1>;
  using LocationsType = std::array<VectorD, NumLocations>;
  using WeightsType = std::array<double, NumLocations>;

  /** The dimension of the parent domain. */
  static constexpr int natural_dimension() { return NaturalDimension; }

  /** The number of quadrature locations. */
  static constexpr int num_quadrature_points() { return NumLocations; }

  /** The position in parent coordinates of all quadrature points. */
  const LocationsType& get_points() const { return points_; }

  /** The weight of all quadrature points. */
  const WeightsType& get_weights() const { return weights_; }

  /** The position in parent coordinates of the q-th quadrature point. */
  const VectorD& get_point(int q) const {
    DRAKE_ASSERT(0 <= q && q < NumLocations);
    return points_[q];
  }

  /** The weight of the q-th quadrature point. */
  double get_weight(int q) const {
    DRAKE_ASSERT(0 <= q && q < NumLocations);
    return weights_[q];
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Quadrature);

  /** Construct a Quadrature object given the quadrature locations and the
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
      std::is_base_of<Quadrature<ObjectType::natural_dimension(),
                                 ObjectType::num_quadrature_points()>,
                      ObjectType>::value;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
