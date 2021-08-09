/// @file
///
/// This file contains tests for an acrobot in a multibody plant.  The tests
/// do not depend on the which parser is used, as different parsers should
/// create the same plant from a correct model in the appropriate format
/// (URDF/SDF).

#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/parsing/test/test_loaders.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

class AcrobotModelTests :
      public testing::TestWithParam<test::ModelLoadFunction> {
 public:
  void SetUp() override {
    const std::string base_name = "drake/multibody/benchmarks/acrobot/acrobot";
    auto load_model = GetParam();
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    load_model(base_name, plant_.get(), nullptr);
    // We are done adding models.
    plant_->Finalize();

    ASSERT_TRUE(plant_->HasJointNamed("ShoulderJoint"));
    ASSERT_TRUE(plant_->HasJointNamed("ElbowJoint"));
    ASSERT_TRUE(plant_->HasJointActuatorNamed("ElbowJoint"));

    shoulder_ = &plant_->GetJointByName<RevoluteJoint>("ShoulderJoint");
    elbow_ = &plant_->GetJointByName<RevoluteJoint>("ElbowJoint");
    context_ = plant_->CreateDefaultContext();

    // Create a benchmark model for verification of the parsed model.
    // The benchmark model is created programmatically through Drake's API.
    benchmark_plant_ = benchmarks::acrobot::MakeAcrobotPlant(parameters_, true);
    benchmark_shoulder_ =
        &benchmark_plant_->GetJointByName<RevoluteJoint>("ShoulderJoint");
    benchmark_elbow_ =
        &benchmark_plant_->GetJointByName<RevoluteJoint>("ElbowJoint");
    benchmark_context_ = benchmark_plant_->CreateDefaultContext();
  }

  void VerifyModelMassMatrix(double theta1, double theta2) {
    const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

    const int nv = plant_->num_velocities();

    // Set the state for the benchmark model and compute the mass matrix.
    benchmark_shoulder_->set_angle(benchmark_context_.get(), theta1);
    benchmark_elbow_->set_angle(benchmark_context_.get(), theta2);
    MatrixX<double> M_benchmark(nv, nv);
    benchmark_plant_->CalcMassMatrixViaInverseDynamics(
        *benchmark_context_.get(), &M_benchmark);

    // Set the state for the parsed model and compute the mass matrix.
    shoulder_->set_angle(context_.get(), theta1);
    elbow_->set_angle(context_.get(), theta2);
    MatrixX<double> M(nv, nv);
    plant_->CalcMassMatrixViaInverseDynamics(*context_.get(), &M);

    EXPECT_TRUE(CompareMatrices(
        M, M_benchmark, kTolerance, MatrixCompareType::relative));
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  const RevoluteJoint<double>* shoulder_{nullptr};
  const RevoluteJoint<double>* elbow_{nullptr};
  std::unique_ptr<systems::Context<double>> context_;

  benchmarks::acrobot::AcrobotParameters parameters_;
  std::unique_ptr<MultibodyPlant<double>> benchmark_plant_;
  const RevoluteJoint<double>* benchmark_shoulder_{nullptr};
  const RevoluteJoint<double>* benchmark_elbow_{nullptr};
  std::unique_ptr<systems::Context<double>> benchmark_context_;
};

// Verifies that the default joint limits are ±infinity when no limits are
// specified.
TEST_P(AcrobotModelTests, DefaultJointLimits) {
  EXPECT_TRUE(std::isinf(shoulder_->position_lower_limit()));
  EXPECT_TRUE(std::isinf(shoulder_->position_upper_limit()));
  EXPECT_TRUE(std::isinf(shoulder_->velocity_lower_limit()));
  EXPECT_TRUE(std::isinf(shoulder_->velocity_upper_limit()));
  EXPECT_TRUE(std::isinf(shoulder_->acceleration_lower_limit()));
  EXPECT_TRUE(std::isinf(shoulder_->acceleration_upper_limit()));
  EXPECT_TRUE(std::isinf(elbow_->position_lower_limit()));
  EXPECT_TRUE(std::isinf(elbow_->position_upper_limit()));
  EXPECT_TRUE(std::isinf(elbow_->velocity_lower_limit()));
  EXPECT_TRUE(std::isinf(elbow_->velocity_upper_limit()));
  EXPECT_TRUE(std::isinf(elbow_->acceleration_lower_limit()));
  EXPECT_TRUE(std::isinf(elbow_->acceleration_upper_limit()));
}

// This test verifies a number of invariants such as model sizes and that body
// and joint models were properly added.
TEST_P(AcrobotModelTests, ModelBasics) {
  EXPECT_EQ(benchmark_plant_->num_bodies(), plant_->num_bodies());
  EXPECT_EQ(benchmark_plant_->num_joints(), plant_->num_joints());

  // Verify we parsed the actuated joint correctly.
  EXPECT_EQ(benchmark_plant_->num_actuators(), plant_->num_actuators());
  EXPECT_EQ(benchmark_plant_->num_actuated_dofs(), plant_->num_actuated_dofs());
  EXPECT_EQ(plant_->GetJointActuatorByName("ElbowJoint").joint().index(),
            elbow_->index());

  // Verify we parse damping correctly.
  EXPECT_EQ(shoulder_->damping(), parameters_.b1());
  EXPECT_EQ(elbow_->damping(), parameters_.b2());

  // State size.
  EXPECT_EQ(plant_->num_positions(), benchmark_plant_->num_positions());
  EXPECT_EQ(plant_->num_velocities(), benchmark_plant_->num_velocities());
  EXPECT_EQ(plant_->num_multibody_states(),
            benchmark_plant_->num_multibody_states());

  // Get links by name.
  const Body<double>& link1 = plant_->GetBodyByName(parameters_.link1_name());
  EXPECT_EQ(link1.name(), parameters_.link1_name());
  const Body<double>& link2 = plant_->GetBodyByName(parameters_.link2_name());
  EXPECT_EQ(link2.name(), parameters_.link2_name());

  // Get joints by name.
  const Joint<double>& shoulder_joint = plant_->GetJointByName("ShoulderJoint");
  EXPECT_EQ(shoulder_joint.name(), parameters_.shoulder_joint_name());
  const Joint<double>& elbow_joint = plant_->GetJointByName("ElbowJoint");
  EXPECT_EQ(elbow_joint.name(), parameters_.elbow_joint_name());

  EXPECT_EQ(shoulder_joint.parent_body().index(), world_index());
  EXPECT_EQ(shoulder_joint.child_body().name(), parameters_.link1_name());
  EXPECT_EQ(elbow_joint.parent_body().name(), parameters_.link1_name());
  EXPECT_EQ(elbow_joint.child_body().name(), parameters_.link2_name());

  // Get actuators by name.
  const JointActuator<double>& elbow_actuator =
      plant_->GetJointActuatorByName("ElbowJoint");
  EXPECT_TRUE(std::isinf(elbow_actuator.effort_limit()));

  // Get frames by name.
  const Frame<double>& link1_frame = plant_->GetFrameByName("Link1");
  EXPECT_EQ(link1_frame.name(), "Link1");
  const Frame<double>& link2_frame = plant_->GetFrameByName("Link2");
  EXPECT_EQ(link2_frame.name(), "Link2");
}

// Verify the parsed model computes the same mass matrix as a Drake benchmark
// for a number of arbitrary configurations.
TEST_P(AcrobotModelTests, VerifyMassMatrixAgainstBenchmark) {
  VerifyModelMassMatrix(0.0, 0.0);
  VerifyModelMassMatrix(0.0, M_PI / 3);
  VerifyModelMassMatrix(0.0, 3 * M_PI / 4);
  VerifyModelMassMatrix(0.0, -M_PI / 3);
  VerifyModelMassMatrix(0.0, -3 * M_PI / 4);

  VerifyModelMassMatrix(M_PI / 3, 0.0);
  VerifyModelMassMatrix(M_PI / 3, M_PI / 3);
  VerifyModelMassMatrix(M_PI / 3, 3 * M_PI / 4);
  VerifyModelMassMatrix(M_PI / 3, -M_PI / 3);
  VerifyModelMassMatrix(M_PI / 3, -3 * M_PI / 4);

  VerifyModelMassMatrix(-M_PI / 3, 0.0);
  VerifyModelMassMatrix(-M_PI / 3, M_PI / 3);
  VerifyModelMassMatrix(-M_PI / 3, 3 * M_PI / 4);
  VerifyModelMassMatrix(-M_PI / 3, -M_PI / 3);
  VerifyModelMassMatrix(-M_PI / 3, -3 * M_PI / 4);
}

INSTANTIATE_TEST_SUITE_P(SdfAcrobatModelTests,
                        AcrobotModelTests,
                        ::testing::Values(test::LoadFromSdf));


INSTANTIATE_TEST_SUITE_P(UrdfAcrobatModelTests,
                        AcrobotModelTests,
                        ::testing::Values(test::LoadFromUrdf));


}  // namespace
}  // namespace multibody
}  // namespace drake
