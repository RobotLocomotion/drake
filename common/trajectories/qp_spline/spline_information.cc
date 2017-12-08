#include "drake/common/trajectories/qp_spline/spline_information.h"

#include <vector>

#include "drake/common/drake_assert.h"

SplineInformation::SplineInformation(
    std::vector<int> const& segment_polynomial_orders,
    std::vector<double> const& breaks)
    : PiecewisePolynomialBase(breaks),
      segment_polynomial_degrees(segment_polynomial_orders) {
  DRAKE_ASSERT(breaks.size() == segment_polynomial_orders.size() + 1);
  value_constraints.resize(getNumberOfSegments());
}

void SplineInformation::addValueConstraint(int segment_index,
                                           ValueConstraint const& constraint) {
  value_constraints[segment_index].push_back(constraint);
}

std::vector<ValueConstraint> const& SplineInformation::getValueConstraints(
    int segment_number) const {
  return value_constraints[segment_number];
}

std::vector<ContinuityConstraint> const&
SplineInformation::getContinuityConstraints() const {
  return continuity_constraints;
}

int SplineInformation::getNumberOfConstraints() const {
  size_t ret = 0;
  for (auto it = value_constraints.begin(); it != value_constraints.end();
       ++it) {
    ret += it->size();
  }
  ret += continuity_constraints.size();
  return static_cast<int>(ret);
}

void SplineInformation::addContinuityConstraint(
    ContinuityConstraint const& constraint) {
  continuity_constraints.push_back(constraint);
}

std::vector<double> const& SplineInformation::getSegmentTimes() const {
  return breaks;
}

int SplineInformation::getSegmentPolynomialDegree(int segment_number,
                                                  Eigen::Index,
                                                  Eigen::Index) const {
  checkScalarValued();
  return segment_polynomial_degrees[segment_number];
}

Eigen::Index SplineInformation::rows() const { return 1; }

Eigen::Index SplineInformation::cols() const { return 1; }
