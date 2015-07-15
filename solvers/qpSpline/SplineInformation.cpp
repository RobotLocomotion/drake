#include "SplineInformation.h"
#include <numeric>
#include <cassert>
#include <stdexcept>

using namespace std;

SplineInformation::SplineInformation(std::vector<int> const & segment_polynomial_orders, std::vector<double> const & segment_times) :
  PiecewisePolynomialBase(segment_times),
  segment_polynomial_degrees(segment_polynomial_orders)
{
  assert(segment_times.size() == segment_polynomial_orders.size() + 1);
  value_constraints.resize(getNumberOfSegments());
}

void SplineInformation::addValueConstraint(int segment_index, ValueConstraint const & constraint) {
  value_constraints[segment_index].push_back(constraint);
}

std::vector<ValueConstraint> const & SplineInformation::getValueConstraints(int segment_number) const {
  return value_constraints[segment_number];
}

std::vector<ContinuityConstraint> const & SplineInformation::getContinuityConstraints() const {
  return continuity_constraints;
}

int SplineInformation::getNumberOfConstraints() const {
  size_t ret = 0;
  for (auto it = value_constraints.begin(); it != value_constraints.end(); ++it) {
    ret += it->size();
  }
  ret += continuity_constraints.size();
  return static_cast<int>(ret);
}

void SplineInformation::addContinuityConstraint(ContinuityConstraint const & constraint) {
  continuity_constraints.push_back(constraint);
}

std::vector<double> const & SplineInformation::getSegmentTimes() const {
  return segment_times;
}

int SplineInformation::getSegmentPolynomialDegree(int segment_number, Eigen::DenseIndex row, Eigen::DenseIndex cols) const {
  checkScalarValued();
  return segment_polynomial_degrees[segment_number];
}

Eigen::DenseIndex SplineInformation::rows() const {
  return 1;
}

Eigen::DenseIndex SplineInformation::cols() const {
  return 1;
}
