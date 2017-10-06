#include "drake/manipulation/perception/optitrack_sim.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"

namespace drake {
namespace manipulation {
namespace perception {

using drake::systems::KinematicsResults;

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

class OptitrackSimTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Create the tree.
    tree_ = std::make_unique<RigidBodyTree<double>>();
    auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
        Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    tree_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(FindResourceOrThrow(kIiwaUrdf),
                                                multibody::joints::kFixed,
                                                weld_to_frame, tree_.get());

    // Arbitrarily pick the first three bodies in the iiwa model.
    // There is one RigidBodyFrame per body that will be tracked.
    body_name_to_id_map_["iiwa_link_0"] = 1;
    body_name_to_id_map_["iiwa_link_1"] = 2;
    body_name_to_id_map_["iiwa_link_2"] = 3;

    // Each RigidBodyFrame has a pose offset w.r.t. the RigidBody it is attached
    // to. Here we set up an arbitrarily chosen pose offset for the frames.
    Eigen::Vector3d axis(1 / sqrt(3), 1 / sqrt(3), 1 / sqrt(3));
    T_BF_ = Eigen::AngleAxisd(0.2, axis);

    // Create the RigidBodyFrames associated with the named bodies, and apply
    // the pose offset.
    for (auto it = body_name_to_id_map_.begin();
         it != body_name_to_id_map_.end(); ++it) {
      RigidBody<double>* body = tree_.get()->FindBody(it->first);
      body_frame_to_id_map_[new RigidBodyFrame<double>(
          it->first + "_frame", body, T_BF_)] = it->second;
    }
  }

  std::vector<TrackedObject> UpdateInputCalcOutput(
      const OptitrackSim& dut, const KinematicsResults<double>& input_results) {
    std::unique_ptr<systems::AbstractValue> input(
        new systems::Value<KinematicsResults<double>>(tree_.get()));
    input->SetValue(input_results);

    context_ = dut.CreateDefaultContext();
    output_ = dut.AllocateOutput(*context_);
    context_->FixInputPort(0 /* input port ID*/, std::move(input));

    dut.CalcUnrestrictedUpdate(*context_, context_->get_mutable_state());
    dut.CalcOutput(*context_, output_.get());
    auto output_value = output_->get_data(0);

    return output_value->GetValue<std::vector<TrackedObject>>();
  }

  std::unique_ptr<RigidBodyTree<double>> tree_;
  std::map<std::string, int> body_name_to_id_map_;
  std::map<RigidBodyFrame<double>*, int> body_frame_to_id_map_;
  Eigen::Isometry3d T_BF_;

 private:
  std::unique_ptr<OptitrackSim> dut_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
};

TEST_F(OptitrackSimTest, InvalidBodyIdTest) {
  auto body_name_to_id_map = body_name_to_id_map_;
  body_name_to_id_map["iiwa_link_0"] = 2;  // adds an invalid (repeated) id
  EXPECT_ANY_THROW(OptitrackSim(*tree_.get(), body_name_to_id_map));

  body_name_to_id_map["iiwa_link_0"] = -1;  // adds an invalid (negative) id
  EXPECT_ANY_THROW(OptitrackSim(*tree_.get(), body_name_to_id_map));
}

TEST_F(OptitrackSimTest, InvalidBodyNameTest) {
  auto body_name_to_id_map = body_name_to_id_map_;
  body_name_to_id_map["invalid_body"] = 4;
  EXPECT_ANY_THROW(OptitrackSim(*tree_.get(), body_name_to_id_map));
}

TEST_F(OptitrackSimTest, InvalidFrameTest) {
  // Adds an invalid RigidBodyFrame object with no associated RigidBody
  auto body_frame_to_id_map = body_frame_to_id_map_;
  body_frame_to_id_map[new RigidBodyFrame<double>()] = 4;
  EXPECT_EQ(body_frame_to_id_map.size(), 4);
  EXPECT_ANY_THROW(OptitrackSim(*tree_.get(), body_frame_to_id_map));
}

TEST_F(OptitrackSimTest, ValidBodyNameTest) {
  std::vector<Eigen::Isometry3d> frame_poses(3, T_BF_);
  OptitrackSim dut(*tree_.get(), body_name_to_id_map_, frame_poses);

  // Update the input, calculate the output, and compare it with expected pose.
  KinematicsResults<double> kres(tree_.get());
  VectorX<double> q(7), v(7);      // sets iiwa positions and velocities
  q << 0.3, 0.3, 0.3, 0, 0, 0, 0;  // pick an arbitrary iiwa pose
  kres.Update(q, v.setZero());

  std::vector<TrackedObject> objects = UpdateInputCalcOutput(dut, kres);
  EXPECT_EQ(objects.size(), body_name_to_id_map_.size());

  std::sort(objects.begin(), objects.end());
  for (auto it = body_name_to_id_map_.begin(); it != body_name_to_id_map_.end();
       ++it) {
    // Get the body pose, and the frame pose. Apply the inverse transform to the
    // frame pose and it should match the body pose.
    RigidBody<double>* rbody = tree_.get()->FindBody(it->first);
    Eigen::Isometry3d T_WB = kres.get_pose_in_world(*rbody);
    auto obj_it = std::find_if(objects.begin(), objects.end(),
                               [&it](const TrackedObject& tobj) {
                                 return tobj.optitrack_id == it->second;
                               });
    EXPECT_TRUE(obj_it != objects.end());
    EXPECT_TRUE(T_WB.isApprox(obj_it->T_WF * T_BF_.inverse(), 1e-3));
  }
}

TEST_F(OptitrackSimTest, ValidFramePoseTest) {
  OptitrackSim dut(*tree_.get(), body_frame_to_id_map_);

  // Update the input, calculate the output, and compare it with expected pose.
  KinematicsResults<double> kres(tree_.get());

  VectorX<double> q(7), v(7);      // sets iiwa positions and velocities
  q << 0.3, 0.3, 0.3, 0, 0, 0, 0;  // pick an arbitrary iiwa pose
  kres.Update(q, v.setZero());

  std::vector<TrackedObject> objects = UpdateInputCalcOutput(dut, kres);
  EXPECT_EQ(objects.size(), body_name_to_id_map_.size());

  std::sort(objects.begin(), objects.end());
  for (auto it = body_frame_to_id_map_.begin();
       it != body_frame_to_id_map_.end(); ++it) {
    // Transform the frame pose from body coordinates to world coordinates, and
    // compare the result to the frame transform stored in the TrackedObject.
    // They should match.
    Eigen::Isometry3d T_WB =
        kres.get_pose_in_world(it->first->get_rigid_body());
    auto obj_it = std::find_if(objects.begin(), objects.end(),
                               [&it](const TrackedObject& tobj) {
                                 return tobj.optitrack_id == it->second;
                               });
    EXPECT_TRUE(obj_it != objects.end());
    EXPECT_TRUE(
        obj_it->T_WF.isApprox(T_WB * it->first->get_transform_to_body()));
  }
}

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
