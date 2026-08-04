#pragma once

#include <expected>
#include <memory>
#include <utility>

#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {
namespace analysis_test {

/* A collection of related systems and contexts, plus a concrete integrator,
for use in integrator abstract tests.

@tparam SpecificSystem A (typically leaf) system whose type and instance (by
pointer) will be known to the tests.*/

template <typename SpecificSystem>
struct IntegratorTestArticles {
  /* May be a diagram containing `sub_system` or be a direct (owning) alias of
  `sub_system`. */
  std::unique_ptr<System<double>> root_system;
  /* May be the diagram root context containing `sub_context`, or be a direct
  (owning) alias of `sub_context`. */
  std::unique_ptr<Context<double>> root_context;
  /* The system whose type will be known and exploited by the tests. */
  const SpecificSystem* sub_system{};
  /* The matching context for `sub_system` */
  Context<double>* sub_context{};
  /* An integrator of some derived type (see below), constructed with
  `root_system` and `root_context`. */
  std::unique_ptr<IntegratorBase<double>> integrator;
};

/* Functor to make IntegratorTestArticles for the supplied `Integrator` type
and `SpecificSystem`. On failure, returns a string explaining why. Abstract
tests are expected to display the returned failure string and skip any relevant
tests. The base template will not fail, but custom specializations might need
to.

This base template works for system/diagram-oblivious integrators. Other
integrators will need to supply their own specializations.

Note to specializers: the factory method will be called during
::testing::SetUp(), so it is possible to examine test case information via the
::testing::UnitTest singleton.

@tparam Integrator a derived type from systems::IntegratorBase.
*/
template <typename Integrator>
  requires std::derived_from<Integrator, IntegratorBase<double>>
struct IntegratorTestFactory {
  /* Makes IntegratorTestArticles for `Integrator` and `SpecificSystem`.
  Returns a reason string on failure.
  */
  template <typename SpecificSystem>
  static std::expected<IntegratorTestArticles<SpecificSystem>, std::string_view>
  MakeIntegratorTestArticles(std::unique_ptr<SpecificSystem> system) {
    IntegratorTestArticles<SpecificSystem> result;
    // Root and sub-system are the same object.
    result.sub_system = system.get();
    result.root_system = std::move(system);
    // Root and sub-context are the same object.
    result.root_context = result.sub_system->CreateDefaultContext();
    result.sub_context = result.root_context.get();
    result.integrator = std::make_unique<Integrator>(*result.root_system,
                                                     result.root_context.get());
    return result;
  }
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
