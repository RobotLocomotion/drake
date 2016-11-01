#include "drake/solvers/qpSpline/ValueConstraint.h"

#include "drake/common/drake_assert.h"

ValueConstraint::ValueConstraint(int derivative_order, double time,
                                 double value)
    : derivative_order_(derivative_order), time_(time), value_(value) {
  DRAKE_ASSERT(derivative_order >= 0);
}

int ValueConstraint::getDerivativeOrder() const { return derivative_order_; }

double ValueConstraint::getTime() const { return time_; }

double ValueConstraint::getValue() const { return value_; }
