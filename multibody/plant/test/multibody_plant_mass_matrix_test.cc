#include <limits>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {

using multibody::Parser;
using systems::Context;
using test::LimitMalloc;

namespace multibody {
namespace {

// We verify the computation of the mass matrix by comparing two significantly
// different implementations:
//   - CalcMassMatrix(): uses the Composite Body Algorithm.
//   - CalcMassMatrixViaInverseDynamics(): uses inverse dynamics to compute each
//     column of the mass matrix at a time.
class MultibodyPlantMassMatrixTests : public ::testing::Test {
 public:
  void LoadModel(const std::string& file_path) {
    const std::string model_path = FindResourceOrThrow(file_path);
    Parser parser(&plant_);
    parser.AddModels(model_path);
    plant_.Finalize();
  }

  void LoadIiwaWithGripper() {
    const char kArmSdfPath[] =
        "drake/manipulation/models/iiwa_description/sdf/"
            "iiwa14_no_collision.sdf";

    const char kWsg50SdfPath[] =
        "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf";

    Parser parser(&plant_);
    const ModelInstanceIndex arm_model =
        parser.AddModels(FindResourceOrThrow(kArmSdfPath)).at(0);

    // Add the gripper.
    const ModelInstanceIndex gripper_model =
        parser.AddModels(FindResourceOrThrow(kWsg50SdfPath)).at(0);

    const auto& base_body = plant_.GetBodyByName("iiwa_link_0", arm_model);
    const auto& end_effector = plant_.GetBodyByName("iiwa_link_7", arm_model);
    const auto& gripper_body = plant_.GetBodyByName("body", gripper_model);
    plant_.WeldFrames(plant_.world_frame(), base_body.body_frame());
    plant_.WeldFrames(end_effector.body_frame(), gripper_body.body_frame());
    plant_.Finalize();
  }

  // For a given state stored in `context`, this verifies the computation of the
  // mass matrix by comparing the results from CalcMassMatrix() and
  // CalcMassMatrixViaInverseDynamics().
  void VerifyMassMatrixComputation(const Context<double>& context) {
    // Compute mass matrix via the Composite Body Algorithm.
    MatrixX<double> Mcba(plant_.num_velocities(), plant_.num_velocities());
    {
      // CalcMassMatrix on `double` should never allocate.
      LimitMalloc guard;
      plant_.CalcMassMatrix(context, &Mcba);
    }

    // Compute mass matrix using inverse dynamics for each column.
    MatrixX<double> Mid(plant_.num_velocities(), plant_.num_velocities());
    plant_.CalcMassMatrixViaInverseDynamics(context, &Mid);

    // Compute a suitable tolerance scaled with the norm of the mass matrix.
    // Since .norm() computes the Frobenius norm, and num_velocities() is the
    // squared root of the number of elements in the matrix, this tolerance is
    // effectively being scaled by the RMS value of the elements in the mass
    // matrix.
    const double kTolerance = 10.0 * std::numeric_limits<double>::epsilon() *
                              Mcba.norm() / plant_.num_velocities();
    EXPECT_TRUE(
        CompareMatrices(Mcba, Mid, kTolerance, MatrixCompareType::relative));
  }

 protected:
  MultibodyPlant<double> plant_{0.0};
};

TEST_F(MultibodyPlantMassMatrixTests, IiwaRobot) {
  LoadModel(
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  // We did not weld the arm to the world, therefore we expect it to be free.
  EXPECT_EQ(plant_.num_velocities(), 13);

  // Create a context and store an arbitrary configuration.
  std::unique_ptr<Context<double>> context = plant_.CreateDefaultContext();
  const VectorX<double> q0 = VectorX<double>::LinSpaced(
      plant_.num_positions(), 1, plant_.num_positions());
  plant_.SetPositions(context.get(), q0);
  VerifyMassMatrixComputation(*context);
}

// This Atlas model contains a number of kinematics chains of massless bodies.
// Therefore this test verifies our implementation can handle this situation.
TEST_F(MultibodyPlantMassMatrixTests, AtlasRobot) {
  LoadModel("drake/examples/atlas/urdf/atlas_convex_hull.urdf");

  // Create a context and store an arbitrary configuration.
  std::unique_ptr<Context<double>> context = plant_.CreateDefaultContext();
  for (JointIndex joint_index(0); joint_index < plant_.num_joints();
       ++joint_index) {
    const Joint<double>& joint = plant_.get_joint(joint_index);
    // This model has weld, revolute, and floating joints. Set the revolute
    // joints to an arbitrary angle.
    if (joint.type_name() == RevoluteJoint<double>::kTypeName) {
      const RevoluteJoint<double>& revolute_joint =
          dynamic_cast<const RevoluteJoint<double>&>(joint);
      // Arbitrary non-zero angle.
      revolute_joint.set_angle(context.get(), 0.5 * joint_index);
    }
  }
  VerifyMassMatrixComputation(*context);
}


// Verify that optimizations over fixed (weld) joints don't cause trouble.
// This is to prevent a repeat of the bug introduced in PR #13933 (fixed in
// #13953).
TEST_F(MultibodyPlantMassMatrixTests, AtlasRobotWithFixedJoints) {
  // TODO(sherm1) Replace this large copied file with the original Atlas urdf
  //              plus a patch or in-memory edit (issue #13954).
  LoadModel("drake/multibody/plant/test/atlas_with_fixed_joints.urdf");

  // Create a context and store an arbitrary configuration.
  std::unique_ptr<Context<double>> context = plant_.CreateDefaultContext();
  for (JointIndex joint_index(0); joint_index < plant_.num_joints();
       ++joint_index) {
    const Joint<double>& joint = plant_.get_joint(joint_index);
    // This model has weld, revolute, and floating joints. Set the revolute
    // joints to an arbitrary angle.
    if (joint.type_name() == RevoluteJoint<double>::kTypeName) {
      const RevoluteJoint<double>& revolute_joint =
          dynamic_cast<const RevoluteJoint<double>&>(joint);
      // Arbitrary non-zero angle.
      revolute_joint.set_angle(context.get(), 0.5 * joint_index);
    }
  }
  VerifyMassMatrixComputation(*context);
}

// Here is a realistic example that fails with the #13933 bug due to
// welded-on gripper, but works with the fix in #13953.
TEST_F(MultibodyPlantMassMatrixTests, IiwaWithWeldedGripper) {
  LoadIiwaWithGripper();

  // Create a context and store an arbitrary configuration.
  std::unique_ptr<Context<double>> context = plant_.CreateDefaultContext();
  for (JointIndex joint_index(0); joint_index < plant_.num_joints();
       ++joint_index) {
    const Joint<double>& joint = plant_.get_joint(joint_index);
    // This model only has weld, prismatic, and revolute joints.
    if (joint.type_name() == RevoluteJoint<double>::kTypeName) {
      const RevoluteJoint<double>& revolute_joint =
          dynamic_cast<const RevoluteJoint<double>&>(joint);
      // Arbitrary non-zero angle.
      revolute_joint.set_angle(context.get(), 0.5 * joint_index);
    } else if (joint.type_name() == PrismaticJoint<double>::kTypeName) {
       const PrismaticJoint<double>& prismatic_joint =
          dynamic_cast<const PrismaticJoint<double>&>(joint);
      // Arbitrary non-zero joint translation.
      prismatic_joint.set_translation(context.get(), 0.5 * joint_index);
    }
  }
  VerifyMassMatrixComputation(*context);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
