/* Tests that CENIC behaves like other integrators are expected to, by adapting
 * it to run under the generic integrator tests.
 */
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/test_utilities/cubic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/explicit_error_controlled_integrator_test.h"
#include "drake/systems/analysis/test_utilities/generic_integrator_test.h"
#include "drake/systems/analysis/test_utilities/integrator_test_factory.h"
#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"
#include "drake/systems/analysis/test_utilities/quadratic_scalar_system.h"

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
  static IntegratorTestStuff<SpecificSystem> MakeIntegratorTestStuff(
      std::unique_ptr<SpecificSystem> system) {
    IntegratorTestStuff<SpecificSystem> result;
    DiagramBuilder<double> builder;
    result.sub_system = system.get();
    builder.AddSystem(std::move(system));

    // Maybe add a plant, based on the offered system's compile-time type and
    // its (run-time) discrete/continuous disposition.
    bool plant_needed = true;
    if constexpr (std::is_same_v<SpecificSystem, MultibodyPlant<double>>) {
      if (!result.sub_system->is_discrete()) {
        // Offered system is a continuous plant. No need to add anything.
        plant_needed = false;
      }
    }
    if (plant_needed) {
      auto plant =
          builder.template AddNamedSystem<MultibodyPlant<double>>("plant", 0.0);
      // Add a single free body to the world. This prevents CENIC complaining
      // about having no cliques.
      const double radius = 0.05;  // m
      const double mass = 0.1;     // kg
      multibody::SpatialInertia<double> M_BBcm =
          multibody::SpatialInertia<double>::SolidSphereWithMass(mass, radius);

      plant->AddRigidBody("Ball", M_BBcm);
      plant->Finalize();
    }

    result.root_system = builder.Build();
    result.root_context = result.root_system->CreateDefaultContext();
    result.sub_context = &result.sub_system->GetMyMutableContextFromRoot(
        result.root_context.get());
    result.integrator = std::make_unique<CenicIntegrator<double>>(
        *result.root_system, result.root_context.get());
    // Use tighter than normal accuracy for generic integrator tests.
    result.integrator->set_target_accuracy(1e-10);
    return result;
  }
};

typedef ::testing::Types<IntegratorTestFactory<CenicIntegrator<double>>> Types;
// NOLINTNEXTLINE(whitespace/line_length)
INSTANTIATE_TYPED_TEST_SUITE_P(My, ExplicitErrorControlledIntegratorTest,
                               Types);
INSTANTIATE_TYPED_TEST_SUITE_P(My, PleiadesTest, Types);
INSTANTIATE_TYPED_TEST_SUITE_P(My, GenericIntegratorTest, Types);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
