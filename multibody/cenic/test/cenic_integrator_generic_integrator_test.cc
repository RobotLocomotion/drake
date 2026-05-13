/* Tests that CENIC behaves like other integrators are expected to, by adapting
 * it to run under the generic integrator tests.
 */
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/test_utilities/explicit_error_controlled_integrator_test.h"
#include "drake/systems/analysis/test_utilities/generic_integrator_test.h"
#include "drake/systems/analysis/test_utilities/integrator_test_factory.h"

namespace drake {
namespace systems {
namespace analysis_test {

using multibody::CenicIntegrator;
using multibody::MultibodyPlant;

// Specialize IntegratorTestFactory for CenicIntegrator such that it can be
// used with the generic integrator tests. The root system will be a diagram
// containing the offered system, and maybe an added continuous time plant.
template <>
struct IntegratorTestFactory<CenicIntegrator<double>> {
  template <typename SpecificSystem>
  static std::expected<IntegratorTestArticles<SpecificSystem>, std::string_view>
  MakeIntegratorTestArticles(std::unique_ptr<SpecificSystem> system) {
    // Check for tests to skip.
    const testing::TestInfo* const test_info =
        testing::UnitTest::GetInstance()->current_test_info();
    if (test_info->name() == std::string("SpringMassStepEC") ||
        test_info->name() == std::string("Pleiades")) {
      return std::unexpected(
          "TODO(#23921, #24304): CENIC cannot yet pass this test.");
    }
    if (test_info->name() == std::string("TrivialFixedStepJointsLocked") ||
        test_info->name() ==
            std::string("TrivialErrorControlledStepJointsLocked")) {
      return std::unexpected("TODO(#23764): CENIC cannot yet pass this test.");
    }

    IntegratorTestArticles<SpecificSystem> result;
    DiagramBuilder<double> builder;
    result.sub_system = system.get();
    builder.AddSystem(std::move(system));

    auto system_is_continuous_plant = [&]() -> bool {
      if constexpr (std::is_same_v<SpecificSystem, MultibodyPlant<double>>) {
        return !result.sub_system->is_discrete();
      }
      return false;
    };

    // If `system` is not the plant CENIC needs, then add one.
    if (!system_is_continuous_plant()) {
      auto plant =
          builder.template AddNamedSystem<MultibodyPlant<double>>("plant", 0.0);
      plant->Finalize();
    }

    result.root_system = builder.Build();
    result.root_context = result.root_system->CreateDefaultContext();
    result.sub_context = &result.sub_system->GetMyMutableContextFromRoot(
        result.root_context.get());
    result.integrator = std::make_unique<CenicIntegrator<double>>(
        *result.root_system, result.root_context.get());
    return result;
  }
};

typedef ::testing::Types<IntegratorTestFactory<CenicIntegrator<double>>> Types;
INSTANTIATE_TYPED_TEST_SUITE_P(My, ExplicitErrorControlledIntegratorTest,
                               Types);
INSTANTIATE_TYPED_TEST_SUITE_P(My, PleiadesTest, Types);
INSTANTIATE_TYPED_TEST_SUITE_P(My, GenericIntegratorTest, Types);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
