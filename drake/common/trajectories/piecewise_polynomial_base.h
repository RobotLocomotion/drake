#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/trajectories/piecewise_function.h"

class PiecewisePolynomialBase : public PiecewiseFunction {
 public:
  explicit PiecewisePolynomialBase(std::vector<double> const& segment_times);

  virtual ~PiecewisePolynomialBase();

  virtual int getSegmentPolynomialDegree(int segment_number,
                                         Eigen::Index row = 0,
                                         Eigen::Index col = 0) const = 0;

  int getNumberOfCoefficients(int segment_number, Eigen::Index row = 0,
                              Eigen::Index col = 0) const;

  int getTotalNumberOfCoefficients(Eigen::Index row = 0,
                                   Eigen::Index col = 0) const;

 protected:
  PiecewisePolynomialBase();
};
