#include "drake/automotive/idm_controller.h"

#include "gtest/gtest.h"

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/test/idm_test.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

namespace drake {
namespace automotive {
namespace {

using maliput::dragway::RoadGeometry;

class IdmControllerTest : public IdmTest {
 protected:
  void Initialize() override {
    // Create a straight road with one lane.
    road_.reset(new maliput::dragway::RoadGeometry(
        maliput::api::RoadGeometryId({"Single-Lane Dragway"}),
        1 /* num_lanes */, 100. /* length */, 2. /* lane_width */,
        0. /* shoulder_width */));

    // Initialize IdmController with the road.
    dut_.reset(new IdmController<double>(*road_));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);

    idm_ = dynamic_cast<const IdmController<double>*>(dut_.get());
    DRAKE_DEMAND(idm_ != nullptr);
  }

  int ego_pose_input_index() override {
    return idm_->ego_pose_input().get_index();
  }
  int ego_velocity_input_index() override {
    return idm_->ego_velocity_input().get_index();
  }
  int traffic_input_index() override {
    return idm_->traffic_input().get_index();
  }
  int command_output_index() override {
    return idm_->driving_command_output().get_index();
  }

  const IdmController<double>* idm_{nullptr};
  std::unique_ptr<maliput::dragway::RoadGeometry> road_;
};

TEST_F(IdmControllerTest, Topology) {
  ASSERT_EQ(3, dut_->get_num_input_ports());
  const auto& ego_pose_input_descriptor =
      dut_->get_input_port(ego_pose_input_index());
  EXPECT_EQ(systems::kVectorValued, ego_pose_input_descriptor.get_data_type());
  EXPECT_EQ(7 /* PoseVector input */, ego_pose_input_descriptor.size());
  const auto& ego_velocity_input_descriptor =
      dut_->get_input_port(ego_velocity_input_index());
  EXPECT_EQ(systems::kVectorValued,
            ego_velocity_input_descriptor.get_data_type());
  EXPECT_EQ(6 /* FrameVelocity input */, ego_velocity_input_descriptor.size());
  const auto& traffic_input_descriptor =
      dut_->get_input_port(traffic_input_index());
  EXPECT_EQ(systems::kAbstractValued, traffic_input_descriptor.get_data_type());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_port(command_output_index());
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(2 /* DrivingCommand output */, output_descriptor.size());
}

TEST_F(IdmControllerTest, Output) {
  // Define a pointer to where the DrivingCommand results end up.
  const auto result = output_->get_vector_data(command_output_index());
  const auto command = dynamic_cast<const DrivingCommand<double>*>(result);
  ASSERT_NE(nullptr, command);

  // Perform the IDM test.
  this->TestIdmPlanner(*command);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
