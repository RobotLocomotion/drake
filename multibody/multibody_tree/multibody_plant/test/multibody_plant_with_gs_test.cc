#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <functional>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_system.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"

#include <iostream>
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace multibody_plant {
namespace {

using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::FrameIdVector;
using geometry::FramePoseVector;
using geometry::GeometrySystem;
using multibody::benchmarks::Acrobot;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using systems::AbstractValue;
using systems::Context;
using systems::ContinuousState;
using systems::DiagramBuilder;
using systems::Diagram;

// Fixture to perform a number of computational tests on an acrobot model.
class AcrobotPlantTests : public ::testing::Test {
 public:
  // Creates MultibodyPlant for an acrobot model.
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    geometry_system_ = builder.AddSystem<GeometrySystem>();
    geometry_system_->set_name("geometry_system");
    const AcrobotParameters acrobot_parameters;
    plant_ = builder.AddSystem(MakeAcrobotPlant(parameters_, geometry_system_));
    // Sanity check on the availability of the optional source id before using it.
    DRAKE_DEMAND(!!plant_->get_source_id());
    builder.Connect(
        plant_->get_geometry_ids_output_port(),
        geometry_system_->get_source_frame_id_port(
            plant_->get_source_id().value()));
    builder.Connect(
        plant_->get_geometry_poses_output_port(),
        geometry_system_->get_source_pose_port(plant_->get_source_id().value()));
    // And build the Diagram:
    diagram_ = builder.Build();

    link1_ = &plant_->GetBodyByName(parameters_.link1_name());
    link2_ = &plant_->GetBodyByName(parameters_.link2_name());
    shoulder_ = &plant_->GetJointByName<RevoluteJoint>(
        parameters_.shoulder_joint_name());
    elbow_ = &plant_->GetJointByName<RevoluteJoint>(
        parameters_.elbow_joint_name());

    context_ = plant_->CreateDefaultContext();
    derivatives_ = plant_->AllocateTimeDerivatives();
  }

  // Verifies the computation performed by MultibodyPlant::CalcTimeDerivatives()
  // for the acrobot model. The comparison is carried out against a benchmark
  // with hand written dynamics.
  void VerifyCalcTimeDerivatives(double theta1, double theta2,
                                 double theta1dot, double theta2dot) {
    const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

    // Set the state:
    shoulder_->set_angle(context_.get(), theta1);
    elbow_->set_angle(context_.get(), theta2);
    shoulder_->set_angular_rate(context_.get(), theta1dot);
    elbow_->set_angular_rate(context_.get(), theta2dot);

    plant_->CalcTimeDerivatives(*context_, derivatives_.get());
    const VectorXd xdot = derivatives_->CopyToVector();

    // Now compute inverse dynamics using our benchmark:
    Vector2d C_expected = acrobot_benchmark_.CalcCoriolisVector(
        theta1, theta2, theta1dot, theta2dot);
    Vector2d tau_g_expected =
        acrobot_benchmark_.CalcGravityVector(theta1, theta2);
    Vector2d rhs = tau_g_expected - C_expected;
    Matrix2d M_expected = acrobot_benchmark_.CalcMassMatrix(theta2);
    Vector2d vdot_expected = M_expected.inverse() * rhs;
    VectorXd xdot_expected(4);
    xdot_expected << Vector2d(theta1dot, theta2dot), vdot_expected;

    EXPECT_TRUE(CompareMatrices(
        xdot, xdot_expected, kTolerance, MatrixCompareType::relative));
  }

 protected:
  // The parameters of the model:
  const AcrobotParameters parameters_;
  // The model plant:
  MultibodyPlant<double>* plant_;
  // A GeometrySystem so that we can test geometry registration.
  GeometrySystem<double>* geometry_system_;
  // The Diagram containing both the MultibodyPlant and the GeometrySystem.
  std::unique_ptr<Diagram<double>> diagram_;
  // Workspace including context and derivatives vector:
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  // Non-owning pointers to the model's elements:
  const Body<double>* link1_{nullptr};
  const Body<double>* link2_{nullptr};
  const RevoluteJoint<double>* shoulder_{nullptr};
  const RevoluteJoint<double>* elbow_{nullptr};

  // Reference benchmark for verification.
  Acrobot<double> acrobot_benchmark_{
      Vector3d::UnitZ() /* Plane normal */, Vector3d::UnitY() /* Up vector */,
      parameters_.m1(), parameters_.m2(),
      parameters_.l1(), parameters_.l2(),
      parameters_.lc1(), parameters_.lc2(),
      parameters_.Ic1(), parameters_.Ic2(),
      parameters_.b1(), parameters_.b2(),
      parameters_.g()};
};

TEST_F(AcrobotPlantTests, VerifyGeometryRegistration) {


  EXPECT_EQ(plant_->get_num_visual_geometries(), 3);
  EXPECT_TRUE(plant_->geometry_source_is_registered());
  EXPECT_TRUE(plant_->get_source_id());

  std::unique_ptr<systems::Context<double>> context =
      plant_->CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      plant_->AllocateOutput(*context);

  std::unique_ptr<AbstractValue> ids_value =
      plant_->get_geometry_ids_output_port().Allocate(*context);
  EXPECT_NO_THROW(ids_value->GetValueOrThrow<FrameIdVector>());
  const FrameIdVector& ids = ids_value->GetValueOrThrow<FrameIdVector>();
  EXPECT_EQ(ids.get_source_id(), plant_->get_source_id());
  EXPECT_EQ(ids.size(), 2);  // Only two frames move.

  std::unique_ptr<AbstractValue> poses_value =
      plant_->get_geometry_poses_output_port().Allocate(*context);
  EXPECT_NO_THROW(poses_value->GetValueOrThrow<FramePoseVector<double>>());
  const FramePoseVector<double>& poses =
      poses_value->GetValueOrThrow<FramePoseVector<double>>();
  EXPECT_EQ(poses.get_source_id(), plant_->get_source_id());
  EXPECT_EQ(poses.vector().size(), 2);  // Only two frames move.

  //shoulder_->set_angle(context.get(), 0.0);
 // elbow_->set_angle(context.get(), 0.0);

  PRINT_VARn(context->get_continuous_state_vector().CopyToVector());

  plant_->get_geometry_poses_output_port().Calc(*context, poses_value.get());

  PRINT_VARn(poses.vector()[0].matrix());
  PRINT_VARn(poses.vector()[1].matrix());
}


}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

