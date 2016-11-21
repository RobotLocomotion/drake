#pragma once

#include <vector>

#include "drake/common/trajectories/piecewise_polynomial_base.h"
#include "drake/common/trajectories/qp_spline/continuity_constraint.h"
#include "drake/common/trajectories/qp_spline/value_constraint.h"

class SplineInformation : public PiecewisePolynomialBase {
 private:
  std::vector<int> segment_polynomial_degrees;
  std::vector<std::vector<ValueConstraint> > value_constraints;
  std::vector<ContinuityConstraint> continuity_constraints;

 public:
  virtual ~SplineInformation() {}

  SplineInformation(std::vector<int> const& segment_polynomial_orders,
                    std::vector<double> const& segment_times);

  virtual int getSegmentPolynomialDegree(int segment_number, Eigen::Index row,
                                         Eigen::Index cols) const;

  virtual Eigen::Index rows() const;

  virtual Eigen::Index cols() const;

  std::vector<ValueConstraint> const& getValueConstraints(
      int segment_number) const;

  std::vector<ContinuityConstraint> const& getContinuityConstraints() const;

  void addValueConstraint(int segment_index, ValueConstraint const& constraint);

  int getNumberOfConstraints() const;

  void addContinuityConstraint(ContinuityConstraint const& constraint);

  std::vector<double> const& getSegmentTimes() const;
};
