#pragma once

#include "drake/common/drake_export.h"

class DRAKE_EXPORT ContinuityConstraint {
 public:
  ContinuityConstraint(int first_spline_index, int second_spline_index,
                       int derivative_order);
  int getDerivativeOrder() const;
  int getFirstSplineIndex() const;
  int getSecondSplineIndex() const;

 private:
  int derivative_order_;
  int first_spline_index_;
  int second_spline_index_;
};
