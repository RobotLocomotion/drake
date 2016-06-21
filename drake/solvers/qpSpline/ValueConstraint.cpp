#include "ValueConstraint.h"

#include "drake/common/drake_assert.h"

ValueConstraint::ValueConstraint(int derivative_order, double time,
                                 double value)
    : derivative_order(derivative_order), time(time), value(value) {
  DRAKE_ASSERT(derivative_order >= 0);
}

int ValueConstraint::getDerivativeOrder() const { return derivative_order; }

double ValueConstraint::getTime() const { return time; }

double ValueConstraint::getValue() const { return value; }
