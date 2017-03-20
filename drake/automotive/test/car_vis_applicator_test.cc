#include "drake/automotive/car_vis_applicator.h"

#include <memory>
#include <utility>
#include <stdexcept>

#include <Eigen/Dense>
#include <gtest/gtest.h>

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

GTEST_TEST(BoxCarVisTest, BasicTest) {
  const int kModelInstanceId = 1600;
  const std::string kName = "Alice";

  // Instantiates the device under test (DUT).
  BoxCarVis<double> dut(kModelInstanceId, kName);

  const vector<lcmt_viewer_link_data>& vis_elements = dut.GetVisElements();
  EXPECT_EQ(vis_elements.size(), 1u);

  const lcmt_viewer_link_data& link_data = vis_elements.at(0);
  EXPECT_EQ(link_data.name, kName);
  EXPECT_EQ(link_data.robot_num, kModelInstanceId);
  EXPECT_EQ(link_data.num_geom, 1);

  const lcmt_viewer_geometry_data& geom_data = link_data.geom.at(0);
  const int kExpectedGeomType = lcmt_viewer_geometry_data::BOX;
  EXPECT_EQ(geom_data.type, kExpectedGeomType);

  PoseVector<double> root_pose;
  root_pose.set_translation({1, 2, 3});
  root_pose.set_rotation({0, 0, 0, -1});
  const systems::rendering::PoseBundle<double> vis_poses =
      dut.GetPoses(root_pose.get_isometry());
  EXPECT_EQ(vis_poses.get_num_poses(), 1);

  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  {
    expected_pose.translation().x() = 1;
    expected_pose.translation().y() = 2;
    expected_pose.translation().z() = 3;
    expected_pose.rotate(Eigen::Quaterniond({0, 0, 0, -1}));
  }
  // The following tolerance was emperically determined.
  EXPECT_TRUE(CompareMatrices(vis_poses.get_pose(0).matrix(),
                              expected_pose.matrix(), 1e-15 /* tolerance */));

  EXPECT_EQ(vis_poses.get_model_instance_id(0), kModelInstanceId);
  EXPECT_EQ(vis_poses.get_name(0), kName);
}

class CarVisApplicatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new CarVisApplicator<double>());
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
  }

  void SetInput(const PoseBundle<double>& pose_bundle) {
    ASSERT_NE(dut_, nullptr);
    ASSERT_NE(context_, nullptr);
    const int kPoseIndex = dut_->get_pose_input_port().get_index();
    std::unique_ptr<AbstractValue> v(
        new Value<PoseBundle<double>>(pose_bundle));
    context_->FixInputPort(kPoseIndex, std::move(v));
  }

  const PoseBundle<double>& GetOutput() const {
    const int kOutputIndex = dut_->get_output_port().get_index();
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
  const auto& pose_port_descriptor = dut_->get_pose_input_port();
  EXPECT_EQ(pose_port_descriptor.get_data_type(), systems::kAbstractValued);

  ASSERT_EQ(dut_->get_num_output_ports(), 1);
  const auto& output_port_descriptor = dut_->get_output_port();
  EXPECT_EQ(output_port_descriptor.get_data_type(), systems::kAbstractValued);

  EXPECT_TRUE(dut_->has_any_direct_feedthrough());
}

TEST_F(CarVisApplicatorTest, Configuration) {
  unique_ptr<CarVis<double>> car_vis_1(new BoxCarVis<double>(kIdZero, "Alice"));
  EXPECT_NO_THROW(dut_->AddCarVis(move(car_vis_1)));

  // Verifies that an exception is thrown if a CarVis object is provided that is
  // a duplicate or out of order.
  unique_ptr<CarVis<double>> car_vis_dup(new BoxCarVis<double>(kIdZero, "Bob"));
  EXPECT_THROW(dut_->AddCarVis(move(car_vis_dup)), std::runtime_error);
  unique_ptr<CarVis<double>> out_of_order_car_vis(
      new BoxCarVis<double>(5 /* out of order model instance ID */, "Foo"));
  EXPECT_THROW(dut_->AddCarVis(move(out_of_order_car_vis)), std::runtime_error);

  // Tests CarVisApplicator::get_load_robot_message().
  const lcmt_viewer_load_robot& load_message = dut_->get_load_robot_message();
  EXPECT_EQ(load_message.num_links, 1);
  EXPECT_EQ(load_message.link.at(0).name, "Alice");
  EXPECT_EQ(load_message.link.at(0).robot_num, kIdZero);
  EXPECT_EQ(load_message.link.at(0).num_geom, 1);
  const int kExpectedGeomType = lcmt_viewer_geometry_data::BOX;
  EXPECT_EQ(load_message.link.at(0).geom.at(0).type, kExpectedGeomType);

  EXPECT_EQ(dut_->num_cars(), 1);
  EXPECT_EQ(dut_->num_vis_poses(), 1);
}

TEST_F(CarVisApplicatorTest, InputOutput) {
  unique_ptr<CarVis<double>> car_vis_1(new BoxCarVis<double>(kIdZero, "Alice"));
  EXPECT_NO_THROW(dut_->AddCarVis(move(car_vis_1)));

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
  unique_ptr<CarVis<double>> car_vis_1(new BoxCarVis<double>(kIdZero, "Alice"));
  EXPECT_NO_THROW(dut_->AddCarVis(move(car_vis_1)));

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
