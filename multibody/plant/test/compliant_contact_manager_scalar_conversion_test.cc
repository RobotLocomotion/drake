#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::systems::Context;
using drake::systems::Simulator;
using Eigen::VectorXd;

template <typename T>
class CompliantContactManagerScalarConversionTest : public ::testing::Test {};

using NonSymbolicScalars = ::testing::Types<double, AutoDiffXd>;

TYPED_TEST_SUITE(CompliantContactManagerScalarConversionTest,
                 NonSymbolicScalars);

TYPED_TEST(CompliantContactManagerScalarConversionTest, ToDouble) {
  using T = TypeParam;
  CompliantContactManager<T> source;
  EXPECT_TRUE(source.is_cloneable_to_double());
  std::unique_ptr<DiscreteUpdateManager<double>> clone =
      source.template CloneToScalar<double>();
  ASSERT_NE(clone, nullptr);
}

TYPED_TEST(CompliantContactManagerScalarConversionTest, ToAutoDiffXd) {
  using T = TypeParam;
  CompliantContactManager<T> source;
  EXPECT_TRUE(source.is_cloneable_to_autodiff());
  std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>> clone =
      source.template CloneToScalar<AutoDiffXd>();
  ASSERT_NE(clone, nullptr);
}

TYPED_TEST(CompliantContactManagerScalarConversionTest, ToSymbolic) {
  using T = TypeParam;
  CompliantContactManager<T> source;
  EXPECT_TRUE(source.is_cloneable_to_symbolic());
  std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>> clone =
      source.template CloneToScalar<symbolic::Expression>();
  ASSERT_NE(clone, nullptr);
}

constexpr double kTimeStep = 0.001;

// Constructs a plant with a free rigid body and uses the SAP solver.
// The argument `solver_type` allows to exercise the TAMSI and SAP solver
// pipelines.
template <typename T>
std::unique_ptr<MultibodyPlant<T>> MakePlant(
    DiscreteContactSolver solver_type) {
  auto plant = std::make_unique<MultibodyPlant<T>>(kTimeStep);
  plant->AddRigidBody("Body", SpatialInertia<double>::MakeUnitary());
  // N.B. We want to exercise the TAMSI and SAP code paths. Therefore we
  // arbitrarily choose two model approximations to accomplish this.
  switch (solver_type) {
    case DiscreteContactSolver::kTamsi:
      plant->set_discrete_contact_approximation(
          DiscreteContactApproximation::kTamsi);
      break;
    case DiscreteContactSolver::kSap:
      plant->set_discrete_contact_approximation(
          DiscreteContactApproximation::kSap);
      break;
  }
  plant->Finalize();
  return plant;
}

// Tests that a plant with compliant contact manager survives scalar
// conversion from T to U, and that simulations results for models without
// constraints stay the same.
template <typename T, typename U>
void TestPlantConversionAndSimulate(DiscreteContactSolver solver_type) {
  std::unique_ptr<MultibodyPlant<T>> source_plant = MakePlant<T>(solver_type);
  auto source_context = source_plant->CreateDefaultContext();
  const VectorX<T> initial_state =
      source_plant->GetPositionsAndVelocities(*source_context);
  auto source_simulator =
      std::make_unique<Simulator<T>>(*source_plant, std::move(source_context));
  source_simulator->AdvanceTo(10 * kTimeStep);
  const VectorX<T> source_final_state =
      source_plant->GetPositionsAndVelocities(source_simulator->get_context());
  // Verify something interesting happened in the sim.
  EXPECT_FALSE(CompareMatrices(initial_state, source_final_state, 0.001));

  // Scalar convert to U.
  auto dest_plant = systems::System<T>::template ToScalarType<U>(*source_plant);
  auto dest_context = dest_plant->CreateDefaultContext();
  auto dest_simulator =
      std::make_unique<Simulator<U>>(*dest_plant, std::move(dest_context));
  dest_simulator->AdvanceTo(10 * kTimeStep);
  const VectorX<U> dest_final_state =
      dest_plant->GetPositionsAndVelocities(dest_simulator->get_context());
  // Verify that the U results agree with T results.
  EXPECT_TRUE(CompareMatrices(dest_final_state, source_final_state));
}

GTEST_TEST(ScalarConvertAndSimulateTest, PlantWithSap) {
  TestPlantConversionAndSimulate<double, AutoDiffXd>(
      DiscreteContactSolver::kSap);
  TestPlantConversionAndSimulate<AutoDiffXd, double>(
      DiscreteContactSolver::kSap);
}

GTEST_TEST(ScalarConvertAndSimulateTest, PlantWithTamsi) {
  TestPlantConversionAndSimulate<double, AutoDiffXd>(
      DiscreteContactSolver::kTamsi);
  TestPlantConversionAndSimulate<AutoDiffXd, double>(
      DiscreteContactSolver::kTamsi);
}

template <typename T, typename U>
void TestPlantConversion(DiscreteContactSolver solver_type) {
  std::unique_ptr<MultibodyPlant<T>> source_plant = MakePlant<T>(solver_type);
  // Scalar convert to U. Verify the conversion is successful.
  EXPECT_NO_THROW(systems::System<T>::template ToScalarType<U>(*source_plant));
}

// Scalar conversions involving symbolic::Expression are supported, even if
// discrete updates are not for specific solvers.
GTEST_TEST(ScalarConvertTest, ConversionToAndFromSymbolic) {
  // Conversion from double to symbolic.
  TestPlantConversion<double, symbolic::Expression>(
      DiscreteContactSolver::kTamsi);
  TestPlantConversion<double, symbolic::Expression>(
      DiscreteContactSolver::kSap);

  // Conversion from symbolic to double.
  TestPlantConversion<symbolic::Expression, double>(
      DiscreteContactSolver::kTamsi);
  TestPlantConversion<symbolic::Expression, double>(
      DiscreteContactSolver::kSap);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
