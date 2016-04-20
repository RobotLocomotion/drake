#pragma once

#include "drake/drakeSplineGeneration_export.h"

class DRAKESPLINEGENERATION_EXPORT ContinuityConstraint {
 private:
  int derivative_order;
  int first_spline_index;
  int second_spline_index;

 public:
  ContinuityConstraint(int first_spline_index, int second_spline_index,
                       int derivative_order);
  int getDerivativeOrder() const;
  int getFirstSplineIndex() const;
  int getSecondSplineIndex() const;
};
