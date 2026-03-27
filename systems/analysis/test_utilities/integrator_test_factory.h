#pragma once

#include <memory>
#include <utility>

#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {
namespace analysis_test {

template <typename SpecificSystem>
struct IntegratorTestStuff {
  std::unique_ptr<System<double>> root_system;
  std::unique_ptr<Context<double>> root_context;
  SpecificSystem* sub_system{};
  Context<double>* sub_context{};
  std::unique_ptr<IntegratorBase<double>> integrator;
};

// This base template works for system/diagram-oblivious integrators. Other
// integrators will need to supply their own specializations.
template <typename Integrator>
struct IntegratorTestFactory {
  template <typename SpecificSystem>
  static IntegratorTestStuff<SpecificSystem> MakeIntegratorTestStuff(
      std::unique_ptr<SpecificSystem> system) {
    IntegratorTestStuff<SpecificSystem> result;
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
