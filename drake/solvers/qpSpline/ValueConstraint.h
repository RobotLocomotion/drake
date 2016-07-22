#pragma once

#include "drake/drakeSplineGeneration_export.h"

class DRAKESPLINEGENERATION_EXPORT ValueConstraint {
 public:
  ValueConstraint(int derivative_order, double time, double value);
  int getDerivativeOrder() const;
  double getTime() const;
  double getValue() const;

 private:
  int derivative_order_;
  double time_;
  double value_;
};
