#pragma once

#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace hydroelastic_contact {

/// A "rule" (weights and quadrature points) for computing quadrature over
/// triangular domains.
class TriangleQuadratureRule {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TriangleQuadratureRule)
  TriangleQuadratureRule() = default;
  virtual ~TriangleQuadratureRule() {}

  /// Returns the order of this rule.
  int order() const {
    int rule_order = do_order();
    DRAKE_DEMAND(rule_order >= 1);
    return rule_order;
  }

  /// Returns the vector of quadrature points.
  const std::vector<Vector2<double>>& quadrature_points() const {
    return do_quadrature_points();
  }

  /// Returns the vector of weights.
  const std::vector<double>& weights() const {
    return do_weights();
  }

 protected:
  /// Derived classes shall return the order (>= 1) of this rule.
  virtual int do_order() const = 0;

  /// Derived classes shall return the vector of quadrature points. Each of
  /// these Vector2 objects represents the barycentric coordinates of a
  /// triangle (the third barycentric coordinate is implicit: it is the
  /// difference between unity and the sum of the other two coordinates).
  virtual const std::vector<Vector2<double>>& do_quadrature_points() const = 0;

  /// Derived classes shall return the vector of weights. The sum of all
  /// weights must equal 1.0.
  virtual const std::vector<double>& do_weights() const = 0;
};

}  // namespace hydroelastic_contact
}  // namespace multibody
}  // namespace drake
