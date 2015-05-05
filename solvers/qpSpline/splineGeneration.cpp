#include "splineGeneration.h"

#include <Eigen/Dense>
#include <iostream>
#include <sstream>
#include <numeric>
#include <stdexcept>

using namespace std;
using namespace Eigen;

template <typename DerivedM>
void setConstraintMatrixPart(double time, int derivative_order, MatrixBase<DerivedM> & constraint_matrix, double scaling = 1.0)
{
  double time_power = 1.0;
  typename MatrixBase<DerivedM>::Index num_coefficients = constraint_matrix.cols();

  for (int col = derivative_order; col < num_coefficients; col++)
  {
     double column_power = 1.0;
     for (int i = 0; i< derivative_order; i++)
     {
        column_power *= (col - i);
     }
     constraint_matrix(0, col) = scaling * time_power * column_power;
     time_power *= time;
  }
}

PiecewisePolynomial<double> generateSpline(const SplineInformation& spline_information) {
  int num_segments = spline_information.getNumberOfSegments();
  int num_constraints = spline_information.getNumberOfConstraints();
  int num_coefficients = spline_information.getTotalNumberOfCoefficients();

  if (num_constraints != num_coefficients) {
    stringstream msg;
    msg << "Only the case where the number of coefficients equals the number of variables is currently handled." << endl;
    msg << "Number of coefficients: " << num_coefficients << ", Number of constraints: " << num_constraints << endl;
    throw runtime_error(msg.str().c_str());
  }

  MatrixXd constraint_matrix = MatrixXd::Zero(num_constraints, num_coefficients);
  VectorXd right_hand_side(num_constraints); // should get overwritten completely

  int constraint_row_start = 0;
  std::vector<int> segment_col_starts;
  int segment_col_start = 0;
  for (int i = 0; i < num_segments; i++) {
    segment_col_starts.push_back(segment_col_start);
    segment_col_start += spline_information.getNumberOfCoefficients(i);
  }

  // handle value constraints
  for (int i = 0; i < num_segments; i++) {
    std::vector<ValueConstraint> const & value_constraints = spline_information.getValueConstraints(i);
    for (auto it = value_constraints.begin(); it != value_constraints.end(); ++it) {
      const ValueConstraint& constraint = *it;
      int number_of_coefficients = spline_information.getNumberOfCoefficients(i);
      int segment_col_start = segment_col_starts[i];
      auto constraint_matrix_segment_part = constraint_matrix.block<1, Dynamic>(constraint_row_start, segment_col_start, 1, number_of_coefficients);
      double t_local = constraint.getTime() - spline_information.getStartTime(i);
      setConstraintMatrixPart(t_local, constraint.getDerivativeOrder(), constraint_matrix_segment_part);
      right_hand_side(constraint_row_start) = constraint.getValue();
      constraint_row_start += 1;
    }
  }

  // handle continuity constraints
  std::vector<ContinuityConstraint> continuity_constraints = spline_information.getContinuityConstraints();
  for (auto it = continuity_constraints.begin(); it != continuity_constraints.end(); ++it) {
    const ContinuityConstraint& constraint = *it;
    int first_spline_index = constraint.getFirstSplineIndex();
    int number_of_coefficients_1 = spline_information.getNumberOfCoefficients(first_spline_index);
    auto constraint_matrix_segment_part_1 = constraint_matrix.block<1, Dynamic>(constraint_row_start, segment_col_starts[first_spline_index], 1, number_of_coefficients_1);
    double t_local_1 = spline_information.getEndTime(first_spline_index) - spline_information.getStartTime(first_spline_index);
    setConstraintMatrixPart(t_local_1, constraint.getDerivativeOrder(), constraint_matrix_segment_part_1, 1.0);

    int second_spline_index = constraint.getSecondSplineIndex();
    int number_of_coefficients_2 = spline_information.getNumberOfCoefficients(second_spline_index);
    auto constraint_matrix_segment_part_2 = constraint_matrix.block<1, Dynamic>(constraint_row_start, segment_col_starts[second_spline_index], 1, number_of_coefficients_2);
    setConstraintMatrixPart(0.0, constraint.getDerivativeOrder(), constraint_matrix_segment_part_2, -1.0);

    right_hand_side(constraint_row_start) = 0.0;

    constraint_row_start += 1;
  }

  // solve
  auto decomposition = constraint_matrix.colPivHouseholderQr();
  if (!decomposition.isInvertible())
    throw ConstraintMatrixSingularError();
  VectorXd solution = decomposition.solve(right_hand_side);

  // create Polynomials
  std::vector<Polynomial<double>> polynomials;
  for (int i = 0; i < num_segments; i++) {
    auto coefficients = solution.segment(segment_col_starts[i], spline_information.getNumberOfCoefficients(i));
    polynomials.push_back(Polynomial<double>(coefficients));
  }

  // return a PiecewisePolynomial
  return PiecewisePolynomial<double>(polynomials, spline_information.getSegmentTimes());
}

PiecewisePolynomial<double> nWaypointCubicSpline(const vector<double>& segment_times, double x0, double xd0, double xf, double xdf, const Ref<const VectorXd> &xi) {
  const int num_segments = static_cast<int>(xi.size() + 1);
  assert(segment_times.size() == num_segments + 1);

  int polynomial_order = 3;
  vector<int> segment_polynomial_orders;
  for (int i = 0; i < num_segments; ++i) {
    segment_polynomial_orders.push_back(polynomial_order);
  }

  SplineInformation spline_information(segment_polynomial_orders, segment_times);
  spline_information.addValueConstraint(0, ValueConstraint(0, spline_information.getStartTime(0), x0));
  spline_information.addValueConstraint(0, ValueConstraint(1, spline_information.getStartTime(0), xd0));
  spline_information.addValueConstraint(num_segments - 1, ValueConstraint(0, spline_information.getEndTime(num_segments - 1), xf));
  spline_information.addValueConstraint(num_segments - 1, ValueConstraint(1, spline_information.getEndTime(num_segments - 1), xdf));

  int num_knots = num_segments - 1;
  for (int i = 0; i < num_knots; i++) {
    spline_information.addValueConstraint(i+1, ValueConstraint(0, spline_information.getStartTime(i+1), xi(i)));
  }

  for (int i = 0; i < num_knots; i++) {
    for (int derivative_order = 0; derivative_order < 3; derivative_order++) {
      spline_information.addContinuityConstraint(ContinuityConstraint(derivative_order, i, i + 1));
    }
  }

  return generateSpline(spline_information);
}
