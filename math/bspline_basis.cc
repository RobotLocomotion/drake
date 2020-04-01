#include "drake/math/bspline_basis.h"

#include <algorithm>
#include <set>
#include <utility>

#include <fmt/format.h>

#include "drake/common/default_scalars.h"

namespace drake {
namespace math {
namespace {

template <typename T>
std::vector<T> ConstructDefaultKnots(int order, int num_basis_functions,
                                     KnotVectorType type) {
  if (num_basis_functions < order) {
    throw std::invalid_argument(fmt::format(
        "The number of basis functions ({}) should be greater than or "
        "equal to the order ({}).",
        num_basis_functions, order));
  }
  const int num_knots{num_basis_functions + order};
  std::vector<T> knots(num_knots, 0.0);
  switch (type) {
    case KnotVectorType::kClampedUniform: {
      const T knot_interval =
          1.0 / static_cast<double>(num_basis_functions - (order - 1));
      for (int i = order; i < num_knots; ++i) {
        if (i < num_basis_functions) {
          knots.at(i) = knots.at(i - 1) + knot_interval;
        } else {
          knots.at(i) = 1.0;
        }
      }
      break;
    }
    case KnotVectorType::kUniform: {
      for (int i = 0; i < num_knots; ++i) {
        knots.at(i) = static_cast<T>(i - (order - 1));
      }
    }
  }
  return knots;
}
}  // namespace

template <typename T>
BsplineBasis<T>::BsplineBasis(int order, std::vector<T> knots)
    : order_(order),
      num_basis_functions_(knots.size() - order),
      knots_(std::move(knots)) {
  if (static_cast<int>(knots_.size()) < 2 * order) {
    throw std::invalid_argument(
        fmt::format("The number of knots ({}) should be greater than or "
                    "equal to twice the order ({}).",
                    knots_.size(), 2 * order));
  }
  DRAKE_ASSERT(std::is_sorted(knots_.begin(), knots_.end()));
}

template <typename T>
BsplineBasis<T>::BsplineBasis(int order, int num_basis_functions,
                              KnotVectorType type)
    : BsplineBasis<T>(
          order, ConstructDefaultKnots<T>(order, num_basis_functions, type)) {}

template <typename T>
bool BsplineBasis<T>::IsControlPointActive(
    int control_point_index, const std::array<T, 2>& parameter_interval) const {
  // Changing control point P[i] affects the curve on the interval (tᵢ, tᵢ₊ₖ).
  // We want to know if P[i] affects the curve over the interval [tₛ, tₑ]. This
  // is true if
  //   (tᵢ, tᵢ₊ₖ) ∩ [tₛ,  tₑ] ≠ ∅
  // or, equivalently, if
  //   tᵢ ≤ tₑ ∧ tₛ ≤ tᵢ₊ₖ.
  //
  // Reference:
  // http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node17.html

  // Define short-hand references to match Patrikalakis et al.:
  const std::vector<T>& t = knots();
  const int k = order();
  const T& t_s = parameter_interval[0];
  const T& t_e = parameter_interval[1];
  const int i = control_point_index;

  return t[i] <= t_e && t_s <= t[i + k];
}

template <typename T>
std::vector<int> BsplineBasis<T>::ComputeActiveBasisFunctionIndices(
    const std::array<T, 2>& parameter_interval) const {
  DRAKE_ASSERT(parameter_interval[0] <= parameter_interval[1]);
  DRAKE_ASSERT(parameter_interval[0] >= initial_parameter_value());
  DRAKE_ASSERT(parameter_interval[1] <= final_parameter_value());
  std::vector<int> active_control_point_indices{};
  for (int i = 0; i < num_basis_functions(); ++i) {
    if (IsControlPointActive(i, parameter_interval)) {
      active_control_point_indices.push_back(i);
    }
  }
  return active_control_point_indices;
}

template <typename T>
std::vector<int> BsplineBasis<T>::ComputeActiveBasisFunctionIndices(
    const T& parameter_value) const {
  return ComputeActiveBasisFunctionIndices(
      {{parameter_value, parameter_value}});
}

template <typename T>
T BsplineBasis<T>::EvaluateBasisFunctionI(int index, const T& parameter_value) const {
  std::vector<T> delta(num_basis_functions(), 0.0);
  delta[index] = 1.0;
  return EvaluateCurve(delta, parameter_value);
}

template <typename T>
bool BsplineBasis<T>::operator==(const BsplineBasis<T>& other) const {
  return this->order() == other.order() && this->knots() == other.knots();
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class BsplineBasis)

}  // namespace math
}  // namespace drake
