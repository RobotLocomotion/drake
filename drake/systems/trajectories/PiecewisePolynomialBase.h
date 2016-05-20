#pragma once

#include <Eigen/Core>
#include <vector>
#include "PiecewiseFunction.h"
#include "drake/drakeTrajectories_export.h"

class DRAKETRAJECTORIES_EXPORT PiecewisePolynomialBase
    : public PiecewiseFunction {
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
