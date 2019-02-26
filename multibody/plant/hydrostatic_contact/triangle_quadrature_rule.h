#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace hydrostatic {

/// A "rule" (weights and quadrature points) for computing quadrature over
/// triangular domains.
class TriangleQuadratureRule {
 public:
  virtual ~TriangleQuadratureRule() {}

  /// Returns the order of this rule.
  int order() const {
    int rule_order = do_order();
    return rule_order;
  }

  /// Returns the vector of quadrature points.
  const std::vector<Vector2<double>>& quadrature_points() const {
    const std::vector<Vector2<double>>& rule_quadrature_points =
        do_quadrature_points();
    return rule_quadrature_points;
  }

  /// Returns the vector of weights.
  const std::vector<double>& weights() const {
    const std::vector<double>& rule_weights = do_weights();
    return rule_weights;
  }

 protected:
  /// Derived classes shall return the order (>= 1) of this rule.
  virtual int do_order() const = 0;

  /// Derived classes shall return the vector of quadrature points.
  virtual const std::vector<Vector2<double>>& do_quadrature_points() const = 0;

  /// Derived classes shall return the vector of weights.
  virtual const std::vector<double>& do_weights() const = 0;
};

}  // namespace hydrostatic
}  // namespace multibody
}  // namespace drake
