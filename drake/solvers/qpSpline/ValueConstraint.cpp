#include "ValueConstraint.h"
#include <cassert>

ValueConstraint::ValueConstraint(int derivative_order, double time, double value) :
  derivative_order(derivative_order), time(time), value(value)
{
  assert(derivative_order >= 0);
}


int ValueConstraint::getDerivativeOrder() const
{
  return derivative_order;
}

double ValueConstraint::getTime() const
{
  return time;
}

double ValueConstraint::getValue() const
{
  return value;
}
