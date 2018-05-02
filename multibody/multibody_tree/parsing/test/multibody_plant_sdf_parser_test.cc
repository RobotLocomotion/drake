#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {

using Eigen::Vector3d;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::parsing::AddModelFromSdfFile;
using systems::Context;

namespace multibody {
namespace multibody_plant {
namespace {

class AcrobotModelTests : public ::testing::Test {
 public:
  // Creates MultibodyPlant for an acrobot model.
  void SetUp() override {
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/benchmarks/acrobot/acrobot.sdf");
    plant_ = std::make_unique<MultibodyPlant<double>>();
    AddModelFromSdfFile(full_name, plant_.get());
    // We are done adding models.
    plant_->Finalize();

    ASSERT_TRUE(plant_->HasJointNamed("ShoulderJoint"));
    ASSERT_TRUE(plant_->HasJointNamed("ElbowJoint"));

    shoulder_ = &plant_->GetJointByName<RevoluteJoint>("ShoulderJoint");
    elbow_ = &plant_->GetJointByName<RevoluteJoint>("ElbowJoint");
    context_ = plant_->CreateDefaultContext();

    // Create a benchmark model for verification of the parsed model.
    // The benchmark model is created programmatically through Drake's API.
    benchmark_plant_ = MakeAcrobotPlant(parameters_, true);
    benchmark_shoulder_ =
        &plant_->GetJointByName<RevoluteJoint>("ShoulderJoint");
    benchmark_elbow_ = &plant_->GetJointByName<RevoluteJoint>("ElbowJoint");
    benchmark_context_ = benchmark_plant_->CreateDefaultContext();
  }

  void VerifySdfModelMassMatrix(double theta1, double theta2) {
    const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

    const int nv = plant_->num_velocities();

    // Set the state for the benchmark model and compute the mass matrix.
    benchmark_shoulder_->set_angle(benchmark_context_.get(), theta1);
    benchmark_elbow_->set_angle(benchmark_context_.get(), theta2);
    MatrixX<double> M_benchmark(nv, nv);
    benchmark_plant_->model().CalcMassMatrixViaInverseDynamics(
        *benchmark_context_.get(), &M_benchmark);

    // Set the state for the parsed model and compute the mass matrix.
    shoulder_->set_angle(context_.get(), theta1);
    elbow_->set_angle(context_.get(), theta2);
    MatrixX<double> M(nv, nv);
    plant_->model().CalcMassMatrixViaInverseDynamics(*context_.get(), &M);

    EXPECT_TRUE(CompareMatrices(
        M, M_benchmark, kTolerance, MatrixCompareType::relative));
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  const RevoluteJoint<double>* shoulder_{nullptr};
  const RevoluteJoint<double>* elbow_{nullptr};
  std::unique_ptr<systems::Context<double>> context_;

  AcrobotParameters parameters_;
  std::unique_ptr<MultibodyPlant<double>> benchmark_plant_;
  const RevoluteJoint<double>* benchmark_shoulder_{nullptr};
  const RevoluteJoint<double>* benchmark_elbow_{nullptr};
  std::unique_ptr<systems::Context<double>> benchmark_context_;
};

// This test verifies a number of invariants such as model sizes and that body
// and joint models were properly added.
TEST_F(AcrobotModelTests, ModelBasics) {
  // Model Size. Counting the world body, there should be three bodies.
  EXPECT_EQ(benchmark_plant_->num_bodies(), plant_->num_bodies());
  EXPECT_EQ(benchmark_plant_->num_joints(), plant_->num_joints());
  // Even though our benchmark model does, the parsed model has no actuators.
  EXPECT_EQ(0, plant_->num_actuators());
  EXPECT_EQ(0, plant_->num_actuated_dofs());

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
  EXPECT_EQ(shoulder_joint.name(), "ShoulderJoint");
  const Joint<double>& elbow_joint = plant_->GetJointByName("ElbowJoint");
  EXPECT_EQ(elbow_joint.name(), "ElbowJoint");

  EXPECT_EQ(shoulder_joint.parent_body().index(), world_index());
  EXPECT_EQ(shoulder_joint.child_body().name(), parameters_.link1_name());
  EXPECT_EQ(elbow_joint.parent_body().name(), parameters_.link1_name());
  EXPECT_EQ(elbow_joint.child_body().name(), parameters_.link2_name());
}

// Verify the parsed model computes the same mass matrix as a Drake benchmark
// for a number of arbitrary configurations.
TEST_F(AcrobotModelTests, VerifyMassMatrixAgainstBenchmark) {
  VerifySdfModelMassMatrix(0.0, 0.0);
  VerifySdfModelMassMatrix(0.0, M_PI / 3);
  VerifySdfModelMassMatrix(0.0, 3 * M_PI / 4);
  VerifySdfModelMassMatrix(0.0, -M_PI / 3);
  VerifySdfModelMassMatrix(0.0, -3 * M_PI / 4);

  VerifySdfModelMassMatrix(M_PI / 3, 0.0);
  VerifySdfModelMassMatrix(M_PI / 3, M_PI / 3);
  VerifySdfModelMassMatrix(M_PI / 3, 3 * M_PI / 4);
  VerifySdfModelMassMatrix(M_PI / 3, -M_PI / 3);
  VerifySdfModelMassMatrix(M_PI / 3, -3 * M_PI / 4);

  VerifySdfModelMassMatrix(-M_PI / 3, 0.0);
  VerifySdfModelMassMatrix(-M_PI / 3, M_PI / 3);
  VerifySdfModelMassMatrix(-M_PI / 3, 3 * M_PI / 4);
  VerifySdfModelMassMatrix(-M_PI / 3, -M_PI / 3);
  VerifySdfModelMassMatrix(-M_PI / 3, -3 * M_PI / 4);
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

