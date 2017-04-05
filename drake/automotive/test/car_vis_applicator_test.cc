#include "drake/automotive/car_vis_applicator.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/automotive/box_car_vis.h"
#include "drake/automotive/car_vis.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/lcmtypes/drake/lcmt_viewer_link_data.hpp"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

using std::make_unique;
using std::move;
using std::unique_ptr;
using std::vector;

namespace drake {

using systems::AbstractValue;
using systems::Value;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

namespace automotive {
namespace {

class CarVisApplicatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new CarVisApplicator<double>());
  }

  void CreateOutputAndContext() {
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
  }

  void SetInput(const PoseBundle<double>& pose_bundle) {
    ASSERT_NE(dut_, nullptr);
    ASSERT_NE(context_, nullptr);
    const int kPoseIndex = dut_->get_car_poses_input_port().get_index();
    context_->FixInputPort(kPoseIndex,
        systems::AbstractValue::Make<PoseBundle<double>>(pose_bundle));
  }

  const PoseBundle<double>& GetOutput() const {
    const int kOutputIndex =
        dut_->get_visual_geometry_poses_output_port().get_index();
    return output_->get_data(kOutputIndex)->
        GetValueOrThrow<PoseBundle<double>>();
  }

  const int kIdZero{0};
  std::unique_ptr<CarVisApplicator<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
};

TEST_F(CarVisApplicatorTest, Topology) {
  ASSERT_EQ(dut_->get_num_input_ports(), 1);
  const auto& pose_port_descriptor = dut_->get_car_poses_input_port();
  EXPECT_EQ(pose_port_descriptor.get_data_type(), systems::kAbstractValued);

  ASSERT_EQ(dut_->get_num_output_ports(), 1);
  const auto& output_port_descriptor =
      dut_->get_visual_geometry_poses_output_port();
  EXPECT_EQ(output_port_descriptor.get_data_type(), systems::kAbstractValued);

  EXPECT_TRUE(dut_->HasAnyDirectFeedthrough());
}

TEST_F(CarVisApplicatorTest, Configuration) {
  EXPECT_NO_THROW(dut_->AddCarVis(
      make_unique<BoxCarVis<double>>(kIdZero, "Alice")));
  EXPECT_THROW(dut_->AddCarVis(
      make_unique<BoxCarVis<double>>(kIdZero, "Bob")), std::runtime_error);

  const int kFooId{kIdZero + 5};  // Out of order with respect to kIdZero.
  EXPECT_NO_THROW(dut_->AddCarVis(
      make_unique<BoxCarVis<double>>(kFooId, "Foo")));

  // Tests CarVisApplicator::get_load_robot_message().
  const int kExpectedGeomType = lcmt_viewer_geometry_data::BOX;
  const lcmt_viewer_load_robot& load_message = dut_->get_load_robot_message();
  EXPECT_EQ(load_message.num_links, 2);
  const int alice_index = load_message.link.at(0).name == "Alice" ? 0 : 1;
  EXPECT_EQ(load_message.link.at(alice_index).name, "Alice");
  EXPECT_EQ(load_message.link.at(alice_index).robot_num, kIdZero);
  EXPECT_EQ(load_message.link.at(alice_index).num_geom, 1);
  EXPECT_EQ(load_message.link.at(alice_index).geom.at(0).type,
      kExpectedGeomType);
  const int foo_index = load_message.link.at(0).name == "Foo" ? 0 : 1;
  EXPECT_EQ(load_message.link.at(foo_index).name, "Foo");
  EXPECT_EQ(load_message.link.at(foo_index).robot_num, kFooId);
  EXPECT_EQ(load_message.link.at(foo_index).num_geom, 1);
  EXPECT_EQ(load_message.link.at(foo_index).geom.at(0).type,
      kExpectedGeomType);

  EXPECT_EQ(dut_->num_cars(), 2);
  EXPECT_EQ(dut_->num_vis_poses(), 2);
}

TEST_F(CarVisApplicatorTest, InputOutput) {
  EXPECT_NO_THROW(dut_->AddCarVis(
      make_unique<BoxCarVis<double>>(kIdZero, "Alice")));
  CreateOutputAndContext();
  Eigen::Isometry3d test_pose = Eigen::Isometry3d::Identity();
  {
    const Eigen::Vector3d rpy(0.1, 0.5, 1.57);
    const Eigen::Vector3d xyz(4, -5, 6);
    test_pose.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  PoseBundle<double> input_poses(1 /* num poses */);
  input_poses.set_pose(0, test_pose);
  input_poses.set_name(0, "Alice");
  input_poses.set_model_instance_id(0, kIdZero);
  SetInput(input_poses);

  dut_->CalcOutput(*context_, output_.get());

  const PoseBundle<double>& pose_bundle = GetOutput();
  EXPECT_EQ(pose_bundle.get_num_poses(), 1);
  EXPECT_TRUE(CompareMatrices(pose_bundle.get_pose(0).matrix(),
      test_pose.matrix(), 1e-15));
  EXPECT_EQ(pose_bundle.get_name(0), "Alice");
  EXPECT_EQ(pose_bundle.get_model_instance_id(0), kIdZero);
}

// Verifies that a bad PoseBundle input that contains an undefined model
// instance ID or an undefined name results in an exception being thrown.
TEST_F(CarVisApplicatorTest, BadInput) {
  EXPECT_NO_THROW(dut_->AddCarVis(
      make_unique<BoxCarVis<double>>(kIdZero, "Alice")));
  CreateOutputAndContext();
  Eigen::Isometry3d test_pose = Eigen::Isometry3d::Identity();
  {
    const Eigen::Vector3d rpy(0.1, 0.5, 1.57);
    const Eigen::Vector3d xyz(4, -5, 6);
    test_pose.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  // Use a model instance ID that doesn't match the visualizer's ID.
  const int kBadId{kIdZero + 1};
  PoseBundle<double> input_poses(1 /* num poses */);
  input_poses.set_pose(0, test_pose);
  input_poses.set_name(0, "Alice");
  input_poses.set_model_instance_id(0, kBadId);
  SetInput(input_poses);
  EXPECT_THROW(dut_->CalcOutput(*context_, output_.get()), std::runtime_error);

  // Fixes the ID.
  input_poses.set_model_instance_id(0, kIdZero);
  SetInput(input_poses);
  EXPECT_NO_THROW(dut_->CalcOutput(*context_, output_.get()));

  // Breaks the name.
  input_poses.set_name(0, "Bob");
  SetInput(input_poses);
  EXPECT_THROW(dut_->CalcOutput(*context_, output_.get()), std::runtime_error);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
