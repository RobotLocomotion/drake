#include "drake/manipulation/util/frame_pose_tracker.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace util {

using drake::systems::KinematicsResults;
using drake::parsers::ModelInstanceIdTable;
using systems::rendering::PoseBundle;
using manipulation::util::FramePoseTracker;

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

class FramePoseTrackerTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Create the tree.
    tree_ = std::make_unique<RigidBodyTree<double>>();
    auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
        Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    ModelInstanceIdTable mtable = parsers::urdf::AddModelInstanceFromUrdfFile(
        FindResourceOrThrow(kIiwaUrdf),
        multibody::joints::kFixed,
        weld_to_frame, tree_.get());
    model_id_ = mtable.begin()->second;

    // Arbitrarily pick the first three bodies in the iiwa model to attach
    // frames to. Each frame name should be unique.
    frame_info_["iiwa_frame_0"] =
        std::make_pair("iiwa_link_0", model_id_);
    frame_info_["iiwa_frame_1"] =
        std::make_pair("iiwa_link_1", model_id_);
    frame_info_["iiwa_frame_2"] =
        std::make_pair("iiwa_link_2", model_id_);

    // Each RigidBodyFrame has a pose offset w.r.t. the RigidBody it is attached
    // to. Here we set an arbitrarily chosen pose offset for the frames.
    Eigen::Vector3d axis(1 / sqrt(3), 1 / sqrt(3), 1 / sqrt(3));
    T_BF_ = Eigen::AngleAxisd(0.2, axis);

    // Create the RigidBodyFrames associated with the named bodies, and apply
    // the pose offset.
    for (auto frame_info : frame_info_) {
      RigidBody<double>* body = tree_.get()->FindBody(
          frame_info.second.first, "", frame_info.second.second);
      frames_.push_back(std::make_unique<RigidBodyFrame<double>>(
          frame_info.first, body, T_BF_));
    }
  }

  PoseBundle<double> UpdateInputCalcOutput(
      const FramePoseTracker& dut,
      const KinematicsResults<double>& input_results) {
    std::unique_ptr<systems::AbstractValue> input(
        new systems::Value<KinematicsResults<double>>(tree_.get()));
    input->SetValue(input_results);

    auto context_ = dut.CreateDefaultContext();
    auto output_ = dut.AllocateOutput(*context_);
    context_->FixInputPort(
        dut.get_kinematics_input_port_index() /* input port ID*/,
        std::move(input));

    dut.CalcUnrestrictedUpdate(*context_, &context_->get_mutable_state());
    dut.CalcOutput(*context_, output_.get());
    auto output_value =
        output_->get_data(dut.get_pose_bundle_output_port_index());

    return output_value->GetValue<PoseBundle<double>>();
  }

  std::unique_ptr<RigidBodyTree<double>> tree_;
  int model_id_;
  Eigen::Isometry3d T_BF_;

  std::map<std::string, std::pair<std::string, int>> frame_info_;
  std::vector<std::unique_ptr<RigidBodyFrame<double>>> frames_;

 private:
  std::unique_ptr<FramePoseTracker> dut_;
};

TEST_F(FramePoseTrackerTest, InvalidModelInstanceIdTest) {
  frame_info_["iiwa_frame_3"] = std::make_pair("iiwa_link_3", 10);
  EXPECT_EQ(frame_info_.size(), 4);
  EXPECT_ANY_THROW(FramePoseTracker(*tree_.get(), frame_info_));
}

TEST_F(FramePoseTrackerTest, InvalidBodyNameTest) {
  frame_info_["iiwa_frame_3"] = std::make_pair("invalid_body_name", model_id_);
  EXPECT_EQ(frame_info_.size(), 4);
  EXPECT_ANY_THROW(FramePoseTracker(*tree_.get(), frame_info_));
}

TEST_F(FramePoseTrackerTest, InvalidFrameTest) {
  frames_.push_back(
      std::make_unique<RigidBodyFrame<double>>("iiwa_frame_3", nullptr));
  EXPECT_EQ(frames_.size(), 4);
  EXPECT_ANY_THROW(FramePoseTracker(*tree_.get(), &frames_));
  EXPECT_ANY_THROW(FramePoseTracker(*tree_.get(), nullptr));

  std::vector<std::unique_ptr<RigidBodyFrame<double>>> empty_vector;
  EXPECT_ANY_THROW(FramePoseTracker(*tree_.get(), &empty_vector));
}

TEST_F(FramePoseTrackerTest, InvalidFrameNameTest) {
  EXPECT_EQ(frames_.size(), 3);
  frames_[2]->set_name("iiwa_frame_0");  // Repeat first frame name.
  EXPECT_ANY_THROW(FramePoseTracker(*tree_.get(), &frames_));
}

TEST_F(FramePoseTrackerTest, ValidFrameInfoTest) {
  std::vector<Eigen::Isometry3d> frame_poses(3, T_BF_);
  FramePoseTracker dut(*tree_.get(), frame_info_, frame_poses);

  // Update the input, calculate the output, and compare it with expected pose.
  KinematicsResults<double> kres(tree_.get());
  VectorX<double> q(7), v(7);  // Sets iiwa positions and velocities.
  q << 0.3, 0.3, 0.3, 0, 0, 0, 0;  // Pick an arbitrary iiwa pose.
  kres.Update(q, v.setZero());

  PoseBundle<double> frames_bundle = UpdateInputCalcOutput(dut, kres);
  EXPECT_EQ(frames_bundle.get_num_poses(), frames_.size());

  // Create a frame name to index map for easy indexing within the PoseBundle.
  std::map<std::string, int> frame_name_to_index_map;
  for (int i = 0; i < frames_bundle.get_num_poses(); i++) {
    frame_name_to_index_map[frames_bundle.get_name(i)] = i;
  }

  for (auto& frame : frames_) {
    // Get the body pose w.r.t. the world and the frame pose w.r.t. the body.
    // Multiply these two and the result should equal the pose contained in the
    // PoseBundle object, i.e., the frame pose w.r.t. the world.
    Eigen::Isometry3d T_WB =
        kres.get_pose_in_world(frame.get()->get_rigid_body());
    Eigen::Isometry3d T_BF = frame.get()->get_transform_to_body();

    // Extract the appropriate frame from the PoseBundle object.
    int frame_pose_index = frame_name_to_index_map.at(frame.get()->get_name());
    Eigen::Isometry3d T_WF = frames_bundle.get_pose(frame_pose_index);

    EXPECT_TRUE(T_WF.isApprox(T_WB * T_BF, 1e-6));
  }
}

TEST_F(FramePoseTrackerTest, ValidFrameTest) {
  std::size_t num_frames_in = frames_.size();
  FramePoseTracker dut(*tree_.get(), &frames_);
  EXPECT_EQ(frames_.size(), 0);

  // Update the input, calculate the output, and compare it with expected pose.
  KinematicsResults<double> kres(tree_.get());
  VectorX<double> q(7), v(7);  // Sets iiwa positions and velocities.
  q << 0.3, 0.3, 0.3, 0, 0, 0, 0;  // Pick an arbitrary iiwa pose.
  kres.Update(q, v.setZero());

  PoseBundle<double> frames_bundle = UpdateInputCalcOutput(dut, kres);
  EXPECT_EQ(frames_bundle.get_num_poses(), num_frames_in);

  // Create a frame name to index map for easy searching within the PoseBundle.
  std::map<std::string, int> frame_name_to_index_map;
  for (int i = 0; i < frames_bundle.get_num_poses(); i++) {
    frame_name_to_index_map[frames_bundle.get_name(i)] = i;
  }

  std::vector<std::string> frame_names = dut.get_tracked_frame_names();
  for (auto& frame_name : frame_names) {
    // Get the body pose w.r.t. the world and the frame pose w.r.t. the body.
    // Multiply these two and the result should equal the pose contained in the
    // PoseBundle object, i.e., the frame pose w.r.t. the world.
    RigidBodyFrame<double>* frame = dut.get_mutable_frame(frame_name);
    Eigen::Isometry3d T_WB = kres.get_pose_in_world(frame->get_rigid_body());
    Eigen::Isometry3d T_BF = frame->get_transform_to_body();

    // Extract the appropriate frame from the PoseBundle object.
    int frame_pose_index = frame_name_to_index_map.at(frame->get_name());
    Eigen::Isometry3d T_WF = frames_bundle.get_pose(frame_pose_index);

    EXPECT_TRUE(T_WF.isApprox(T_WB * T_BF, 1e-6));
  }
}

}  // namespace util
}  // namespace manipulation
}  // namespace drake
