#include "drake/systems/sensors/optitrack_encoder.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {
namespace sensors {

using systems::rendering::PoseBundle;

class OptitrackEncoderTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    Eigen::Vector3d axis(1 / sqrt(3), 1 / sqrt(3), 1 / sqrt(3));

    // Create a mock PoseBundle with three arbitrarily chosen poses.
    pose_bundle_ = new PoseBundle<double>(3);
    pose_bundle_->set_name(0, "iiwa_frame_0");
    pose_bundle_->set_pose(
        0, Eigen::Isometry3d(Eigen::AngleAxis<double>(0.1, axis)).
            pretranslate(Eigen::Vector3d(0.2, 0.3, 0.7)));
    pose_bundle_->set_name(1, "iiwa_frame_1");
    pose_bundle_->set_pose(
        1, Eigen::Isometry3d(Eigen::AngleAxis<double>(0.2, axis)).
            pretranslate(Eigen::Vector3d(0.5, 0.8, 0.9)));
    pose_bundle_->set_name(2, "iiwa_frame_2");
    pose_bundle_->set_pose(
        2, Eigen::Isometry3d(Eigen::AngleAxis<double>(0.3, axis)).
            pretranslate(Eigen::Vector3d(0.1, 0.4, 0.8)));

    // The OptitrackEncoder class constructor takes a map of unique frame names
    // (that have a match in the PoseBundle input object) to unique Optitrack
    // IDs.
    frame_name_to_id_map_["iiwa_frame_0"] = 1;
    frame_name_to_id_map_["iiwa_frame_1"] = 2;
    frame_name_to_id_map_["iiwa_frame_2"] = 3;

    // Create a set of custom names for our tracked rigid bodies. These are
    // optional. If not provided, rigid body names will match the frame names
    // contained in the PoseBundle input. These names are not necessarily unique
    // (as in the case for the Optitrack application software, e.g., Motive).
    rigid_body_names_.push_back("body_1");
    rigid_body_names_.push_back("body_2");
    rigid_body_names_.push_back("body_3");
  }

  std::vector<TrackedBody> UpdateInputCalcOutput(
      const OptitrackEncoder& dut) {
    std::unique_ptr<systems::AbstractValue> input(
    new systems::Value<PoseBundle<double>>(pose_bundle_->get_num_poses()));
    input->SetValue(*pose_bundle_);

    auto context = dut.CreateDefaultContext();
    auto output = dut.AllocateOutput(*context);
    context->FixInputPort(
        dut.get_pose_bundle_input_port_index() /* input port ID*/,
        std::move(input));

    dut.CalcUnrestrictedUpdate(*context, &context->get_mutable_state());
    dut.CalcOutput(*context, output.get());
    auto output_value =
        output->get_data(dut.get_optitrack_output_port_index());

    return output_value->GetValue<std::vector<TrackedBody>>();
  }

  virtual void TearDown() {
    delete pose_bundle_;
  }

  std::map<std::string, int> frame_name_to_id_map_;
  std::vector<std::string> rigid_body_names_;

 private:
  PoseBundle<double>* pose_bundle_;
};

TEST_F(OptitrackEncoderTest, InvalidOptitrackIdTest) {
  // Repeat the first Optitrack ID for the third frame (therefore invalid).
  frame_name_to_id_map_["iiwa_frame_2"] = 1;
  EXPECT_ANY_THROW(OptitrackEncoder(frame_name_to_id_map_,
                                    std::vector<std::string>()));

  // Make the third Optitrack ID negative (therefore invalid).
  frame_name_to_id_map_["iiwa_frame_2"] = -2;
  EXPECT_ANY_THROW(OptitrackEncoder(frame_name_to_id_map_,
                                    std::vector<std::string>()));
}

// Create a default OptitrackEncoder without specifying custom rigid body names.
// This test checks the assigned rigid body names have a match within the set of
// incoming frame names within the PoseBundle object.
TEST_F(OptitrackEncoderTest, WithoutRigidBodyNamesTest) {
  OptitrackEncoder dut(frame_name_to_id_map_, std::vector<std::string>());
  auto tracked_objects = UpdateInputCalcOutput(dut);

  for (auto it = tracked_objects.begin(); it != tracked_objects.end(); ++it) {
    EXPECT_EQ((*it).id, frame_name_to_id_map_[(*it).body_name]);
  }
}

// Create an OptitrackEncoder and specify custom rigid body names. This test
// implements a consistency check between the output of this leaf system and
// the parameters passed in at object construction. We ensure each TrackedBody
// stores the correct body name, by comparing these against names passed to the
// constructor.
TEST_F(OptitrackEncoderTest, WithRigidBodyNamesTest) {
  OptitrackEncoder dut(frame_name_to_id_map_, rigid_body_names_);
  auto tracked_objects = UpdateInputCalcOutput(dut);

  EXPECT_EQ(frame_name_to_id_map_.size(), rigid_body_names_.size());
  auto rb_name_iter = rigid_body_names_.begin();
  auto fr_map_iter = frame_name_to_id_map_.begin();
  while (fr_map_iter != frame_name_to_id_map_.end() &&
      rb_name_iter != rigid_body_names_.end()) {
    auto obj_it = std::find_if(tracked_objects.begin(), tracked_objects.end(),
                               [&fr_map_iter](const TrackedBody& tobj) {
                                 return tobj.id == fr_map_iter->second;
                               });
    EXPECT_EQ(*rb_name_iter, (*obj_it).body_name);
    fr_map_iter++;
    rb_name_iter++;
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
