#include "drake/systems/analysis/initial_value_problem.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/analysis/initial_value_problem-inl.h"

namespace drake {
namespace systems {

template class InitialValueProblem<double>;

template <typename T>
using PP = trajectories::PiecewisePolynomial<T>;
template PP<double> InitialValueProblem<double>::Approximate<PP<double>>(
    const ApproximationTechnique<PP<double>>&,
    const double&, const SpecifiedValues&) const;

}  // namespace systems
}  // namespace drake
