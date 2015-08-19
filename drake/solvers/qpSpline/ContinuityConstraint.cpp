#include "ContinuityConstraint.h"
#include <cassert>

ContinuityConstraint::ContinuityConstraint(int derivative_order, int first_spline_index, int second_spline_index) :
  derivative_order(derivative_order), first_spline_index(first_spline_index), second_spline_index(second_spline_index)
{
  assert(derivative_order >= 0);
  assert(first_spline_index >= 0);
  assert(second_spline_index >= 0);
  assert(first_spline_index != second_spline_index);
}

int ContinuityConstraint::getDerivativeOrder() const
{
  return derivative_order;
}

int ContinuityConstraint::getFirstSplineIndex() const
{
  return first_spline_index;
}

int ContinuityConstraint::getSecondSplineIndex() const
{
  return second_spline_index;
}
