#include "drake/math/bspline_basis.h"

#include <algorithm>
#include <functional>
#include <set>
#include <utility>

#include <fmt/format.h>

#include "drake/common/default_scalars.h"

namespace drake {
namespace math {
namespace {

template <typename T>
std::vector<T> MakeKnotVector(int order, int num_basis_functions,
                              KnotVectorType type,
                              const T& initial_parameter_value,
                              const T& final_parameter_value) {
  if (num_basis_functions < order) {
    throw std::invalid_argument(fmt::format(
        "The number of basis functions ({}) should be greater than or "
        "equal to the order ({}).",
        num_basis_functions, order));
  }
  DRAKE_DEMAND(initial_parameter_value <= final_parameter_value);
  const int num_knots{num_basis_functions + order};
  std::vector<T> knots(num_knots);
  const T knot_interval = (final_parameter_value - initial_parameter_value) /
                          (num_basis_functions - order + 1.0);
  for (int i = 0; i < num_knots; ++i) {
    if (i < order && type == KnotVectorType::kClampedUniform) {
      knots.at(i) = initial_parameter_value;
    } else if (i >= num_basis_functions &&
               type == KnotVectorType::kClampedUniform) {
      knots.at(i) = final_parameter_value;
    } else {
      knots.at(i) = initial_parameter_value + knot_interval * (i - (order - 1));
    }
  }
  return knots;
}

// This custom comparator is needed to explicitly convert Formula to bool when
// T == Expression.
template <typename T>
bool less_than_with_cast(const T& val, const T& other) {
  return static_cast<bool>(val < other);
}

}  // namespace

template <typename T>
BsplineBasis<T>::BsplineBasis(int order, std::vector<T> knots)
    : order_(order), knots_(std::move(knots)) {
  if (static_cast<int>(knots_.size()) < 2 * order) {
    throw std::invalid_argument(
        fmt::format("The number of knots ({}) should be greater than or "
                    "equal to twice the order ({}).",
                    knots_.size(), 2 * order));
  }
  DRAKE_ASSERT(CheckInvariants());
}

template <typename T>
BsplineBasis<T>::BsplineBasis(int order, int num_basis_functions,
                              KnotVectorType type,
                              const T& initial_parameter_value,
                              const T& final_parameter_value)
    : BsplineBasis<T>(order, MakeKnotVector<T>(order, num_basis_functions, type,
                                               initial_parameter_value,
                                               final_parameter_value)) {}

template <typename T>
std::vector<int> BsplineBasis<T>::ComputeActiveBasisFunctionIndices(
    const std::array<T, 2>& parameter_interval) const {
  DRAKE_ASSERT(parameter_interval[0] <= parameter_interval[1]);
  DRAKE_ASSERT(parameter_interval[0] >= initial_parameter_value());
  DRAKE_ASSERT(parameter_interval[1] <= final_parameter_value());
  const int first_active_index =
      FindContainingInterval(parameter_interval[0]) - order() + 1;
  const int final_active_index = FindContainingInterval(parameter_interval[1]);
  std::vector<int> active_control_point_indices{};
  active_control_point_indices.reserve(final_active_index - first_active_index);
  for (int i = first_active_index; i <= final_active_index; ++i) {
    active_control_point_indices.push_back(i);
  }
  return active_control_point_indices;
}

template <typename T>
template <typename T_control_point>
T_control_point BsplineBasis<T>::EvaluateCurve(
    const std::vector<T_control_point>& control_points,
    const T& parameter_value) const {
  /* This function implements the de Boor algorithm. It uses the notation
  from Patrikalakis et al. [1]. Since the depth of recursion is known
  a-priori, the algorithm is flattened along the lines described in [2] to
  avoid duplicate computations.

    [1] https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node18.html
    [2] De Boor, Carl. "On calculating with B-splines." Journal of
        Approximation theory 6.1 (1972): 50-62.
  */
  DRAKE_DEMAND(static_cast<int>(control_points.size()) ==
               num_basis_functions());
  DRAKE_DEMAND(parameter_value >= initial_parameter_value());
  DRAKE_DEMAND(parameter_value <= final_parameter_value());

  // Define short names to match notation in [1].
  const std::vector<T>& t = knots();
  const T& t_bar = parameter_value;
  const int k = order();

  /* Find the index, 𝑙, of the greatest knot that is less than or equal to
  t_bar and strictly less than final_parameter_value(). */
  const int ell = FindContainingInterval(t_bar);
  // The vector that stores the intermediate de Boor points (the pᵢʲ in [1]).
  std::vector<T_control_point> p(order());
  /* For j = 0, i goes from ell down to ell - (k - 1). Define r such that
  i = ell - r. */
  for (int r = 0; r < k; ++r) {
    const int i = ell - r;
    p.at(r) = control_points.at(i);
  }
  /* For j = 1, ..., k - 1, i goes from ell down to ell - (k - j - 1). Again,
  i = ell - r. */
  for (int j = 1; j < k; ++j) {
    for (int r = 0; r < k - j; ++r) {
      const int i = ell - r;
      // α = (t_bar - t[i]) / (t[i + k - j] - t[i]);
      const T alpha = (t_bar - t.at(i)) / (t.at(i + k - j) - t.at(i));
      p.at(r) = (1.0 - alpha) * p.at(r + 1) + alpha * p.at(r);
    }
  }
  return p.front();
}

template <typename T>
std::vector<int> BsplineBasis<T>::ComputeActiveBasisFunctionIndices(
    const T& parameter_value) const {
  return ComputeActiveBasisFunctionIndices(
      {{parameter_value, parameter_value}});
}

template <typename T>
T BsplineBasis<T>::EvaluateBasisFunctionI(int index,
                                          const T& parameter_value) const {
  std::vector<T> delta(num_basis_functions(), 0.0);
  delta[index] = 1.0;
  return EvaluateCurve(delta, parameter_value);
}

template <typename T>
int BsplineBasis<T>::FindContainingInterval(const T& parameter_value) const {
  DRAKE_ASSERT(parameter_value >= initial_parameter_value());
  DRAKE_ASSERT(parameter_value <= final_parameter_value());
  const std::vector<T>& t = knots();
  const T& t_bar = parameter_value;
  return std::distance(
      t.begin(), std::prev(t_bar < final_parameter_value()
                               ? std::upper_bound(t.begin(), t.end(), t_bar,
                                                  less_than_with_cast<T>)
                               : std::lower_bound(t.begin(), t.end(), t_bar,
                                                  less_than_with_cast<T>)));
}

template <typename T>
boolean<T> BsplineBasis<T>::operator==(const BsplineBasis<T>& other) const {
  if (this->order() == other.order() &&
      this->num_basis_functions() == other.num_basis_functions()) {
    boolean<T> result{true};
    const int num_knots{num_basis_functions() + order()};
    for (int i = 0; i < num_knots; ++i) {
      result = result && (this->knots()[i] == other.knots()[i]);
      if (std::equal_to<boolean<T>>{}(result, boolean<T>{false})) {
        break;
      }
    }
    return result;
  } else {
    return boolean<T>{false};
  }
}

template <typename T>
boolean<T> BsplineBasis<T>::operator!=(const BsplineBasis<T>& other) const {
  return !this->operator==(other);
}

template <typename T>
bool BsplineBasis<T>::CheckInvariants() const {
  return std::is_sorted(knots_.begin(), knots_.end(), less_than_with_cast<T>) &&
         static_cast<int>(knots_.size()) >= 2 * order_;
}
}  // namespace math
}  // namespace drake

// Explicit instantiations for EvaluateCurve.
#define BSPLINE_INSTANTIATE_EVALUATE_CURVE(T, T_control_point)  \
  template T_control_point                                      \
  drake::math::BsplineBasis<T>::EvaluateCurve<T_control_point>( \
      const std::vector<T_control_point>&, const T&) const;

#define BSPLINE_INSTANTIATE_EVALUATE_CURVE_S(T, S)         \
  BSPLINE_INSTANTIATE_EVALUATE_CURVE(T, S)                 \
  BSPLINE_INSTANTIATE_EVALUATE_CURVE(T, drake::VectorX<S>) \
  BSPLINE_INSTANTIATE_EVALUATE_CURVE(T, drake::MatrixX<S>)

BSPLINE_INSTANTIATE_EVALUATE_CURVE_S(double, double)
BSPLINE_INSTANTIATE_EVALUATE_CURVE_S(double, drake::AutoDiffXd)
BSPLINE_INSTANTIATE_EVALUATE_CURVE_S(drake::AutoDiffXd, drake::AutoDiffXd)
BSPLINE_INSTANTIATE_EVALUATE_CURVE_S(double, drake::symbolic::Expression)
BSPLINE_INSTANTIATE_EVALUATE_CURVE_S(drake::symbolic::Expression,
                                     drake::symbolic::Expression)

#undef BSPLINE_INSTANTIATE_EVALUATE_CURVE
#undef BSPLINE_INSTANTIATE_EVALUATE_CURVE_S

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::math::BsplineBasis);
