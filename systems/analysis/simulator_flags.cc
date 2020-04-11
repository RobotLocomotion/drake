#include "drake/systems/analysis/simulator_flags.h"

#include <cctype>
#include <initializer_list>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/unused.h"
#include "drake/systems/analysis/bogacki_shampine3_integrator.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/radau_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/runge_kutta5_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/velocity_implicit_euler_integrator.h"

namespace drake {
namespace systems {
namespace {

using std::function;
using std::pair;
using std::string;
using std::vector;
using symbolic::Expression;

// A functor that implements ResetIntegrator.
template <typename T>
using ResetIntegratorFunc =
    function<IntegratorBase<T>*(Simulator<T>*, const T& /* max_step_size */)>;

// Returns (scheme, functor) pair that implements ResetIntegrator.
template <typename T>
using NamedResetIntegratorFunc =
    pair<string, ResetIntegratorFunc<T>>;

// Converts the class name of the `Integrator` template argument into a string
// name for the scheme, e.g., FooBarIntegrator<double> becomes "foo_bar".
template <template <typename> class Integrator>
string GetIntegratorName() {
  // Get the class name, e.g., FooBarIntegrator<double>.
  string full_name = NiceTypeName::Get<Integrator<double>>();
  string class_name = NiceTypeName::RemoveNamespaces(full_name);
  if (class_name == "RadauIntegrator<double,1>") {
    class_name = "Radau1Integrator<double>";
  } else if (class_name == "RadauIntegrator<double,2>") {
    class_name = "Radau3Integrator<double>";
  }

  // Strip off "Integrator<double>" suffix to leave just "FooBar".
  const string suffix = "Integrator<double>";
  DRAKE_DEMAND(class_name.size() > suffix.size());
  const size_t suffix_begin = class_name.size() - suffix.size();
  DRAKE_DEMAND(class_name.substr(suffix_begin) == suffix);
  const string camel_name = class_name.substr(0, suffix_begin);

  // Convert "FooBar to "foo_bar".
  string result;
  for (char ch : camel_name) {
    if (std::isupper(ch)) {
      if (!result.empty()) { result.push_back('_'); }
      result.push_back(std::tolower(ch));
    } else {
      result.push_back(ch);
    }
  }
  return result;
}

// Returns (scheme, functor) pair to implement reset for this `Integrator`.
// This would be much simpler if all integrators accepted a max_step_size.
template <typename T, template <typename> class Integrator>
NamedResetIntegratorFunc<T> MakeResetter() {
  constexpr bool is_fixed_step = std::is_constructible_v<
      Integrator<T>,
      const System<T>&, T, Context<T>*>;
  constexpr bool is_error_controlled = std::is_constructible_v<
      Integrator<T>,
      const System<T>&, Context<T>*>;
  static_assert(is_fixed_step ^ is_error_controlled);
  return NamedResetIntegratorFunc<T>(
      GetIntegratorName<Integrator>(),
      [](Simulator<T>* simulator, const T& max_step_size) {
        if constexpr (is_fixed_step) {
          IntegratorBase<T>& result =
              simulator->template reset_integrator<Integrator<T>>(
                  max_step_size);
          return &result;
        } else {
          IntegratorBase<T>& result =
              simulator->template reset_integrator<Integrator<T>>();
          result.set_maximum_step_size(max_step_size);
          return &result;
        }
      });
}

// Returns the full list of supported (scheme, functor) pairs.  N.B. The list
// here must be kept in sync with the help string in simulator_gflags.cc.
template <typename T>
const vector<NamedResetIntegratorFunc<T>>& GetAllNamedResetIntegratorFuncs() {
  static const never_destroyed<vector<NamedResetIntegratorFunc<T>>> result{
    std::initializer_list<NamedResetIntegratorFunc<T>>{
      // Keep this list sorted alphabetically.
      MakeResetter<T, BogackiShampine3Integrator>(),
      MakeResetter<T, ExplicitEulerIntegrator>(),
      MakeResetter<T, ImplicitEulerIntegrator>(),
      MakeResetter<T, Radau1Integrator>(),
      MakeResetter<T, Radau3Integrator>(),
      MakeResetter<T, RungeKutta2Integrator>(),
      MakeResetter<T, RungeKutta3Integrator>(),
      MakeResetter<T, RungeKutta5Integrator>(),
      MakeResetter<T, SemiExplicitEulerIntegrator>(),
      MakeResetter<T, VelocityImplicitEulerIntegrator>(),
  }};
  return result.access();
}

}  // namespace

template <typename T>
IntegratorBase<T>& ResetIntegratorFromFlags(
    Simulator<T>* simulator,
    const string& scheme,
    const T& max_step_size) {
  DRAKE_THROW_UNLESS(simulator != nullptr);

  const auto& name_func_pairs = GetAllNamedResetIntegratorFuncs<T>();
  for (const auto& [one_name, one_func] : name_func_pairs) {
    if (scheme == one_name) {
      return *one_func(simulator, max_step_size);
    }
  }
  throw std::runtime_error(fmt::format(
      "Unknown integration scheme: {}", scheme));
}

const vector<string>& GetIntegrationSchemes() {
  static const never_destroyed<vector<string>> result{[]() {
    vector<string> names;
    const auto& name_func_pairs = GetAllNamedResetIntegratorFuncs<double>();
    for (const auto& [one_name, one_func] : name_func_pairs) {
      names.push_back(one_name);
      unused(one_func);
    }
    return names;
  }()};
  return result.access();
}

// Explicit instantiations.
// We can't support T=Expression because Simulator doesn't support it.
template IntegratorBase<double>& ResetIntegratorFromFlags(
    Simulator<double>*, const string&, const double&);
template IntegratorBase<AutoDiffXd>& ResetIntegratorFromFlags(
    Simulator<AutoDiffXd>*, const string&, const AutoDiffXd&);

}  // namespace systems
}  // namespace drake
