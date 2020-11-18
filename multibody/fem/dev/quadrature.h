#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace multibody {
namespace fem {
/** A class for quadratures that facilitates numerical integrations in FEM. The
 specification of particular quadrature rules will be provided in the derived
 classes.
 @tparam T the scalar type of the function being integrated over.
 @tparam NaturalDimension dimension of the domain of integration.
 */
template <typename T, int NaturalDimension>
class Quadrature {
 public:
  static_assert(1 <= NaturalDimension && NaturalDimension <= 3,
                  "Only 1, 2 and 3 dimensional spaces are supported.");

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Quadrature);

  using VectorD = Eigen::Matrix<T, NaturalDimension, 1>;

  /** The dimension of the parent domain. */
  static constexpr int kNaturalDim = NaturalDimension;

  virtual ~Quadrature() = default;

  /// The number of quadrature points for the quadrature rule.
  int num_points() const { return points_.size(); }

  /// The position in parent coordinates of all quadrature points.
  const std::vector<VectorD>& get_points() const { return points_; }

  /// The position in parent coordinates of the q-th quadrature point.
  const VectorD& get_point(int q) const {
    DRAKE_DEMAND(q >= 0);
    DRAKE_DEMAND(q < num_points());
    return points_[q];
  }

  /// The weight of the q-th quadrature point.
  T get_weight(int q) const {
    DRAKE_DEMAND(q >= 0);
    DRAKE_DEMAND(q < num_points());
    return weights_[q];
  }

 protected:
  explicit Quadrature(
      std::pair<std::vector<VectorD>, std::vector<T>>&& points_and_weights)
      : points_(std::move(points_and_weights.first)),
        weights_(std::move(points_and_weights.second)) {}

 private:
  std::vector<VectorD> points_;
  std::vector<T> weights_;
};

/** Calculates the Gaussian quadrature rule for 2D and 3D unit simplices
 (triangles and tetrahedrons up to cubic order as described section 3 in
 [Hammer, 1956] as well as section 9.10 of [Zienkiewics, 2005]. The 2D unit
 triangle has vertices located at (0,0), (1,0) and (0,1). The 3D unit
 tetrahedron has vertices located at (0,0,0), (1,0,0), (0,1,0) and (0,0,1).
 @tparam QuadratureOrder order of the quadrature rule. Must be 1, 2, or 3. The
 quadrature rule will be exact for polynomials of degree less than or equal to
 QuadratureOrder.
 @tparam NaturalDimension dimension of the unit simplex. Must be 2, or 3.

 [Hammer, 1956] P.C. Hammer, O.P. Marlowe, and A.H. Stroud. Numerical
 integration over simplexes and cones. Math. Tables Aids Comp. 10, 130-7, 1956.
 [Zienkiewicz, 2005] Zienkiewicz, Olek C., Robert L. Taylor, and Jian Z. Zhu.
 The finite element method: its basis and fundamentals. Elsevier, 2005. */
template <typename T, int QuadratureOrder, int NaturalDimension>
class SimplexGaussianQuadrature : public Quadrature<T, NaturalDimension> {
 public:
  using VectorD = typename Quadrature<T, NaturalDimension>::VectorD;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SimplexGaussianQuadrature);
  SimplexGaussianQuadrature()
      : Quadrature<T, NaturalDimension>(ComputePointsAndWeights()) {}

 private:
  std::pair<std::vector<VectorD>, std::vector<T>> ComputePointsAndWeights() {
    static_assert(1 <= QuadratureOrder && QuadratureOrder <= 3,
                  "Only linear, quadratic and cubic quadratures are supported");
    if constexpr (NaturalDimension == 2) {
      // For a unit triangle, area = 0.5.
      if constexpr (QuadratureOrder == 1) {
        // quadrature point location,  weight/area
        //  (1/3, 1/3)                     1.0
        std::vector<VectorD> points = {{1.0 / 3.0, 1.0 / 3.0}};
        std::vector<T> weights = {0.5};
        return std::make_pair(std::move(points), std::move(weights));
        // TODO(xuchenhan-tri): fix the bug in cpplint as described in
        // https://github.com/google/styleguide/issues/541. A solution has been
        // proposed at https://github.com/cpplint/cpplint/pull/136.
        // NOLINTNEXTLINE(readability/braces)
      } else if constexpr (QuadratureOrder == 2) {
        // quadrature point location,  weight/area
        //  (1/6, 1/6)                     1/3
        //  (2/3, 1/6)                     1/3
        //  (1/6, 2/3)                     1/3
        // Note: Here we choose r=1/2 in section 3 of [Hammer, 1956]. They also
        // mentioned the other choice with r=-1/2. We do not use r=-1/2 as it
        // lies out side of the element.
        std::vector<VectorD> points(3);
        points[0] = {1.0 / 6.0, 1.0 / 6.0};
        points[1] = {2.0 / 3.0, 1.0 / 6.0};
        points[2] = {1.0 / 6.0, 2.0 / 3.0};
        std::vector<T> weights = {1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0};
        return std::make_pair(std::move(points), std::move(weights));
        // NOLINTNEXTLINE(readability/braces)
      } else if constexpr (QuadratureOrder == 3) {
        // quadrature point location,  weight/area
        //  (1/3, 1/3)                     -9/16
        //  (3/5, 1/5)                     25/48
        //  (1/5, 3/5)                     25/48
        //  (1/5, 1/5)                     25/48
        std::vector<VectorD> points(4);
        points[0] = {1.0 / 3.0, 1.0 / 3.0};
        points[1] = {0.6, 0.2};
        points[2] = {0.2, 0.6};
        points[3] = {0.2, 0.2};
        std::vector<T> weights = {-9.0 / 32.0, 25.0 / 96.0, 25.0 / 96.0,
                                  25.0 / 96.0};
        return std::make_pair(std::move(points), std::move(weights));
      } else {
        DRAKE_UNREACHABLE();
      }
      // NOLINTNEXTLINE(readability/braces)
    } else if constexpr (NaturalDimension == 3) {
      // For a unit tetrahedron, area = 1/6.
      if constexpr (QuadratureOrder == 1) {
        // quadrature point location,  weight/area
        //  (1/4, 1/4, 1/4)                1.0
        std::vector<VectorD> points = {{0.25, 0.25, 0.25}};
        std::vector<T> weights = {1.0 / 6.0};
        return std::make_pair(std::move(points), std::move(weights));
        // NOLINTNEXTLINE(readability/braces)
      } else if constexpr (QuadratureOrder == 2) {
        // quadrature point location,  weight/area
        //  (a, b, b)                      1/4
        //  (b, a, b)                      1/4
        //  (b, b, a)                      1/4
        //  (b, b, b)                      1/4
        // where a = (1+3*sqrt(1/5))/4, b = (1-1/sqrt(1/5))/4.
        std::vector<VectorD> points(4);
        T a = (1.0 + 3.0 * std::sqrt(0.2)) / 4.0;
        T b = (1.0 - std::sqrt(0.2)) / 4.0;
        points[0] = {a, b, b};
        points[1] = {b, a, b};
        points[2] = {b, b, a};
        points[3] = {b, b, b};
        std::vector<T> weights = {1.0 / 24.0, 1.0 / 24.0, 1.0 / 24.0,
                                  1.0 / 24.0};
        return std::make_pair(std::move(points), std::move(weights));
        // NOLINTNEXTLINE(readability/braces)
      } else if constexpr (QuadratureOrder == 3) {
        // quadrature point location,  weight/area
        //  (1/4, 1/4, 1/4)               -4/5
        //  (a, b, b)                      9/20
        //  (b, a, b)                      9/20
        //  (b, b, a)                      9/20
        //  (b, b, b)                      9/20
        // where a = 1/2, b = 1/6.
        std::vector<VectorD> points(5);
        T a = 0.5;
        T b = 1.0 / 6.0;
        points[0] = {0.25, 0.25, 0.25};
        points[1] = {a, b, b};
        points[2] = {b, a, b};
        points[3] = {b, b, a};
        points[4] = {b, b, b};
        std::vector<T> weights = {-2.0 / 15.0, 3.0 / 40.0, 3.0 / 40.0,
                                  3.0 / 40.0, 3.0 / 40.0};
        return std::make_pair(std::move(points), std::move(weights));
      } else {
        DRAKE_UNREACHABLE();
      }
    } else {
      DRAKE_UNREACHABLE();
    }
  }
};

template <class QuadratureType, typename T = void, int QuadratureOrder = 0,
          int NaturalDimension = 0>
struct is_quadrature {
  static constexpr bool value = false;
};

template <typename T, int QuadratureOrder, int NaturalDimension>
struct is_quadrature<
    SimplexGaussianQuadrature<T, QuadratureOrder, NaturalDimension>> {
  static constexpr bool value = true;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
