#include "drake/solvers/qpSpline/ContinuityConstraint.h"

#include "drake/common/drake_assert.h"

ContinuityConstraint::ContinuityConstraint(int derivative_order,
                                           int first_spline_index,
                                           int second_spline_index)
    : derivative_order_(derivative_order),
      first_spline_index_(first_spline_index),
      second_spline_index_(second_spline_index) {
  DRAKE_ASSERT(derivative_order >= 0);
  DRAKE_ASSERT(first_spline_index >= 0);
  DRAKE_ASSERT(second_spline_index >= 0);
  DRAKE_ASSERT(first_spline_index != second_spline_index);
}

int ContinuityConstraint::getDerivativeOrder() const {
  return derivative_order_;
}

int ContinuityConstraint::getFirstSplineIndex() const {
  return first_spline_index_;
}

int ContinuityConstraint::getSecondSplineIndex() const {
  return second_spline_index_;
}
