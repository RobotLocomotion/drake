#pragma once

#include "drake/common/drake_export.h"

class DRAKE_EXPORT ValueConstraint {
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
