#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/discrete_contact_data.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/multibody/plant/dummy_physical_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/test_utilities/robot_model.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using multibody::RevoluteSpring;
using multibody::test::RobotModel;
using multibody::test::RobotModelConfig;
using symbolic::Expression;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::System;

namespace multibody {
namespace {

template <typename T>
class MultibodyPlantDefaultScalarsTest : public ::testing::Test {
 public:
  MultibodyPlantDefaultScalarsTest() = default;
};

TYPED_TEST_SUITE_P(MultibodyPlantDefaultScalarsTest);

// We test that we can scalar convert a plant containing a revolute joint and
// spring. In particular, we verify that the spring element correctly references
// the joint both before and after scalar conversion.
TYPED_TEST_P(MultibodyPlantDefaultScalarsTest, RevoluteJointAndSpring) {
  using U = TypeParam;

  MultibodyPlant<double> plant(0.0);
  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  const RigidBody<double>& body =
      plant.AddRigidBody("Body", SpatialInertia<double>::MakeUnitary());
  const RevoluteJoint<double>& pin = plant.AddJoint<RevoluteJoint>(
      "Pin", plant.world_body(), std::nullopt, body, std::nullopt,
      Vector3<double>::UnitZ());
  const auto& spring = plant.AddForceElement<RevoluteSpring>(pin, 0, 1500);

  // We verify the correct reference to the pin joint before conversion.
  EXPECT_EQ(&pin, &spring.joint());

  // We are done defining the model.
  plant.Finalize();

  // Sanity check for the model's size.
  DRAKE_DEMAND(plant.num_velocities() == 1);
  DRAKE_DEMAND(plant.num_positions() == 1);
  // The plant has a UniformGravityFieldElement by default plus the
  // RevoluteSpring.
  DRAKE_DEMAND(plant.num_force_elements() == 2);

  std::unique_ptr<MultibodyPlant<U>> plant_u;
  EXPECT_NO_THROW(plant_u = System<double>::ToScalarType<U>(plant));
  EXPECT_EQ(plant.num_velocities(), plant_u->num_velocities());
  EXPECT_EQ(plant.num_positions(), plant_u->num_positions());
  EXPECT_EQ(plant.num_force_elements(), plant_u->num_force_elements());

  // We verify the correct reference to the pin joint after conversion.
  const auto& pin_u =
      plant_u->template GetJointByName<RevoluteJoint>(pin.name());
  const auto& spring_u =
      plant_u->template GetForceElement<RevoluteSpring>(spring.index());

  // Verify correct cross-referencing in the scalar converted model.
  EXPECT_EQ(&pin_u, &spring_u.joint());
}

// Verifies that two MultibodyPlants have the same port indices.
template <typename T, typename U>
void CompareMultibodyPlantPortIndices(const MultibodyPlant<T>& plant_t,
                                      const MultibodyPlant<U>& plant_u) {
  // Check input ports.
  // (Except actuation input ports because there is no actuation source.)
  EXPECT_EQ(plant_t.get_applied_generalized_force_input_port().get_index(),
            plant_u.get_applied_generalized_force_input_port().get_index());
  EXPECT_EQ(plant_t.get_applied_spatial_force_input_port().get_index(),
            plant_u.get_applied_spatial_force_input_port().get_index());
  EXPECT_EQ(plant_t.get_geometry_query_input_port().get_index(),
            plant_u.get_geometry_query_input_port().get_index());
  // Check output ports.
  EXPECT_EQ(plant_t.get_body_poses_output_port().get_index(),
            plant_u.get_body_poses_output_port().get_index());
  EXPECT_EQ(plant_t.get_body_spatial_velocities_output_port().get_index(),
            plant_u.get_body_spatial_velocities_output_port().get_index());
  EXPECT_EQ(plant_t.get_body_spatial_accelerations_output_port().get_index(),
            plant_u.get_body_spatial_accelerations_output_port().get_index());
  EXPECT_EQ(plant_t.get_state_output_port().get_index(),
            plant_u.get_state_output_port().get_index());
  EXPECT_EQ(plant_t.get_generalized_acceleration_output_port().get_index(),
            plant_u.get_generalized_acceleration_output_port().get_index());
  EXPECT_EQ(plant_t.get_reaction_forces_output_port().get_index(),
            plant_u.get_reaction_forces_output_port().get_index());
  EXPECT_EQ(plant_t.get_contact_results_output_port().get_index(),
            plant_u.get_contact_results_output_port().get_index());
  EXPECT_EQ(plant_t.get_geometry_pose_output_port().get_index(),
            plant_u.get_geometry_pose_output_port().get_index());
  EXPECT_EQ(
      plant_t.get_deformable_body_configuration_output_port().get_index(),
      plant_u.get_deformable_body_configuration_output_port().get_index());
  EXPECT_EQ(
      plant_t.get_state_output_port(default_model_instance()).get_index(),
      plant_u.get_state_output_port(default_model_instance()).get_index());
  EXPECT_EQ(
      plant_t.get_generalized_acceleration_output_port(default_model_instance())
          .get_index(),
      plant_u.get_generalized_acceleration_output_port(default_model_instance())
          .get_index());
  EXPECT_EQ(
      plant_t
          .get_generalized_contact_forces_output_port(default_model_instance())
          .get_index(),
      plant_u
          .get_generalized_contact_forces_output_port(default_model_instance())
          .get_index());
}

// This test verifies that the port indices of the input/output ports of
// MultibodyPlant remain the same after scalar conversion.
TYPED_TEST_P(MultibodyPlantDefaultScalarsTest, PortIndexOrdering) {
  using U = TypeParam;

  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  std::unique_ptr<Diagram<U>> diagram_u =
      System<double>::ToScalarType<U>(*diagram);
  const auto& plant_u =
      dynamic_cast<const MultibodyPlant<U>&>(*diagram_u->GetSystems().at(0));

  CompareMultibodyPlantPortIndices(plant, plant_u);
}

// Verifies that we can AddMultibodyPlantSceneGraph, without any conversion.
TYPED_TEST_P(MultibodyPlantDefaultScalarsTest, DirectlyAdded) {
  using U = TypeParam;
  systems::DiagramBuilder<U> builder;
  MultibodyPlant<U>& plant = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  std::unique_ptr<Diagram<U>> diagram = builder.Build();
  diagram->CreateDefaultContext();
}

REGISTER_TYPED_TEST_SUITE_P(MultibodyPlantDefaultScalarsTest,
                            RevoluteJointAndSpring, PortIndexOrdering,
                            DirectlyAdded);

using NonDoubleScalarTypes = ::testing::Types<AutoDiffXd, Expression>;
INSTANTIATE_TYPED_TEST_SUITE_P(My, MultibodyPlantDefaultScalarsTest,
                               NonDoubleScalarTypes);

// A simple concrete DiscreteUpdateManager that does not support scalar
// conversion to either AutoDiffXd or Expression.
template <typename T>
class DoubleOnlyDiscreteUpdateManager final
    : public multibody::internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DoubleOnlyDiscreteUpdateManager);
  DoubleOnlyDiscreteUpdateManager() = default;
  ~DoubleOnlyDiscreteUpdateManager() final = default;

  bool is_cloneable_to_double() const final { return true; }

 private:
  std::unique_ptr<multibody::internal::DiscreteUpdateManager<double>>
  CloneToDouble() const final {
    auto clone = std::make_unique<DoubleOnlyDiscreteUpdateManager<double>>();
    return clone;
  }

  void DoCalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const final {}

  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      internal::AccelerationKinematicsCache<T>*) const final {}

  void DoCalcDiscreteValues(const systems::Context<T>&,
                            systems::DiscreteValues<T>*) const final {}

  void DoCalcDiscreteUpdateMultibodyForces(
      const systems::Context<T>& context,
      MultibodyForces<T>* forces) const final {}

  void DoCalcActuation(const systems::Context<T>&, VectorX<T>*) const final {}
};

// This test verifies that adding external components that do not support some
// scalar types removes MultibodyPlant's ability to scalar convert to those
// scalar types.
GTEST_TEST(ScalarConversionTest, ExternalComponent) {
  MultibodyPlant<double> plant(0.1);
  std::unique_ptr<internal::DummyPhysicalModel<double>> dummy_physical_model =
      std::make_unique<internal::DummyPhysicalModel<double>>(&plant);
  EXPECT_TRUE(dummy_physical_model->is_cloneable_to_double());
  EXPECT_TRUE(dummy_physical_model->is_cloneable_to_autodiff());
  EXPECT_TRUE(dummy_physical_model->is_cloneable_to_symbolic());
  plant.AddDummyModel(std::move(dummy_physical_model));
  plant.Finalize();

  // double -> AutoDiffXd
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff;
  EXPECT_NO_THROW(plant_autodiff = System<double>::ToAutoDiffXd(plant));
  // AutoDiffXd -> double
  EXPECT_NO_THROW(plant_autodiff->ToScalarType<double>());
  // double -> Expression
  std::unique_ptr<MultibodyPlant<Expression>> plant_double_to_symbolic;
  EXPECT_NO_THROW(plant_double_to_symbolic = System<double>::ToSymbolic(plant));
  // double -> Expression
  std::unique_ptr<MultibodyPlant<Expression>> plant_autodiff_to_symbolic;
  EXPECT_NO_THROW(plant_autodiff_to_symbolic =
                      System<AutoDiffXd>::ToSymbolic(*plant_autodiff));

  // Verify that adding a component that doesn't allow scalar conversion to
  // autodiff does not prevent scalar conversion to double.
  auto discrete_update_manager =
      std::make_unique<DoubleOnlyDiscreteUpdateManager<AutoDiffXd>>();
  EXPECT_TRUE(discrete_update_manager->is_cloneable_to_double());
  EXPECT_FALSE(discrete_update_manager->is_cloneable_to_autodiff());
  EXPECT_FALSE(discrete_update_manager->is_cloneable_to_symbolic());
  plant_autodiff->SetDiscreteUpdateManager(std::move(discrete_update_manager));
  EXPECT_NO_THROW(plant_autodiff->ToScalarType<double>());
}

}  // namespace

class MultibodyPlantTester {
 public:
  template <typename T>
  static std::map<MultibodyConstraintId, internal::CouplerConstraintSpec>&
  get_mutable_specs(MultibodyPlant<T>* plant) {
    return plant->coupler_constraints_specs_;
  }
};

namespace {

// Verify that constraint specs survive scalar conversions. Here we only test
// that the number of constraints before and aftter scalar conversion are the
// same. The correctness of the scalar copying semantics for the constraints are
// tested in their own unit tests.
GTEST_TEST(ScalarConversionTest, CouplerConstraintSpec) {
  MultibodyPlant<double> plant_double(0.1);

  const JointIndex j0(3);
  const JointIndex j1(5);
  constexpr double kGearRatio = 1.2;
  constexpr double kOffset = 0.3;
  const internal::CouplerConstraintSpec reference_spec{j0, j1, kGearRatio,
                                                       kOffset};

  // Directly add dummy constraint specs through the tester so that we don't
  // need to actually add any joints.
  MultibodyPlantTester::get_mutable_specs(
      &plant_double)[MultibodyConstraintId::get_new_id()] = reference_spec;
  plant_double.Finalize();
  // double -> AutoDiffXd.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_double_to_autodiff =
      System<double>::ToAutoDiffXd(plant_double);
  EXPECT_EQ(plant_double_to_autodiff->num_constraints(),
            plant_double.num_constraints());
  // AutoDiffXd -> double.
  std::unique_ptr<MultibodyPlant<double>> plant_autodiff_to_double =
      System<AutoDiffXd>::ToScalarType<double>(*plant_double_to_autodiff);
  EXPECT_EQ(plant_autodiff_to_double->num_constraints(),
            plant_double.num_constraints());
}

template <typename T>
class DiscretePlantTest
    : public ::testing::TestWithParam<
          std::tuple<DiscreteContactApproximation, ContactModel,
                     RobotModelConfig::ContactConfig>> {
 public:
  void SetUp() override {
    const auto [contact_approximation, contact_model, contact_configuration] =
        GetParam();
    const RobotModelConfig robot_config{contact_configuration,
                                        contact_approximation, contact_model};
    auto model_double = std::make_unique<RobotModel<double>>(robot_config);
    if constexpr (std::is_same_v<T, double>) {
      model_ = std::move(model_double);
    } else {
      model_ = model_double->ToScalarType<T>();
    }
  }

 protected:
  std::unique_ptr<RobotModel<T>> model_;
};

template <typename T>
std::string ParamInfoToString(
    const testing::TestParamInfo<typename DiscretePlantTest<T>::ParamType>&
        param_info) {
  const auto [contact_approximation, contact_model, contact_configuration] =
      param_info.param;
  const RobotModelConfig robot_config{contact_configuration,
                                      contact_approximation, contact_model};
  std::stringstream s;
  s << robot_config;
  return s.str();
}

// Helper to make all parameter permutations for DiscretePlantTest.
auto MakeAllPermutations() {
  return ::testing::Combine(
      ::testing::Values(DiscreteContactApproximation::kSimilar,
                        DiscreteContactApproximation::kTamsi),
      ::testing::Values(ContactModel::kPoint,
                        ContactModel::kHydroelasticWithFallback),
      ::testing::Values(RobotModelConfig::ContactConfig::kNoGeometry,
                        RobotModelConfig::ContactConfig::kNoContactState,
                        RobotModelConfig::ContactConfig::kInContactState));
}

using DiscretePlantTestDouble = DiscretePlantTest<double>;
using DiscretePlantTestAutoDiff = DiscretePlantTest<AutoDiffXd>;
using DiscretePlantTestExpression = DiscretePlantTest<symbolic::Expression>;

INSTANTIATE_TEST_SUITE_P(SupportMatrixTests, DiscretePlantTestDouble,
                         MakeAllPermutations(), ParamInfoToString<double>);

TEST_P(DiscretePlantTestDouble, ForcedUpdate) {
  const auto& diagram = model_->diagram();
  auto updates = diagram.AllocateDiscreteVariables();
  EXPECT_NO_THROW(diagram.CalcForcedDiscreteVariableUpdate(model_->context(),
                                                           updates.get()));
}

INSTANTIATE_TEST_SUITE_P(SupportMatrixTests, DiscretePlantTestAutoDiff,
                         MakeAllPermutations(), ParamInfoToString<AutoDiffXd>);

TEST_P(DiscretePlantTestAutoDiff, ForcedUpdate) {
  const auto& diagram = model_->diagram();
  auto updates = diagram.AllocateDiscreteVariables();
  EXPECT_NO_THROW(diagram.CalcForcedDiscreteVariableUpdate(model_->context(),
                                                           updates.get()));
}

INSTANTIATE_TEST_SUITE_P(SupportMatrixTests, DiscretePlantTestExpression,
                         MakeAllPermutations(),
                         ParamInfoToString<symbolic::Expression>);

TEST_P(DiscretePlantTestExpression, ForcedUpdate) {
  const auto& diagram = model_->diagram();
  auto context = diagram.CreateDefaultContext();
  // We don't support discrete updates when T = Expression. Depending on the
  // plant configuration, the throw will happen from different places (with
  // slightly different messages). We check two key elements of the message:
  // - not supported
  // - due to Expression
  DRAKE_EXPECT_THROWS_MESSAGE(
      diagram.ExecuteForcedEvents(context.get()),
      ".* (doesn't support|does not support|not supported)"
      ".* (T ?= ?|scalar type )[drake:symbolic]*Expression.*");
}

}  // namespace
}  // namespace multibody
}  // namespace drake
