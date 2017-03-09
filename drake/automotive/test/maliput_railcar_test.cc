#include "drake/automotive/maliput_railcar.h"

#include <cmath>
#include <memory>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {

using maliput::monolane::ArcOffset;
using maliput::monolane::Endpoint;
using maliput::monolane::EndpointXy;
using maliput::monolane::EndpointZ;

using systems::BasicVector;
using systems::LeafContext;
using systems::Parameters;
using systems::rendering::PoseVector;

namespace automotive {
namespace {

class MaliputRailcarTest : public ::testing::Test {
 protected:
  void InitializeDragwayLane() {
    // Defines the dragway's parameters.
    const int kNumLanes{1};
    const double kDragwayLength{50};
    const double kDragwayLaneWidth{0.5};
    const double kDragwayShoulderWidth{0.25};
    Initialize(std::make_unique<const maliput::dragway::RoadGeometry>(
        maliput::api::RoadGeometryId({"RailcarTestDragway"}), kNumLanes,
        kDragwayLength, kDragwayLaneWidth, kDragwayShoulderWidth));
  }

  void InitializeCurvedMonoLane() {
    maliput::monolane::Builder builder(
        maliput::api::RBounds(-2, 2),   /* lane_bounds       */
        maliput::api::RBounds(-4, 4),   /* driveable_bounds  */
        0.01,                           /* linear tolerance  */
        0.5 * M_PI / 180.0);            /* angular_tolerance */
    builder.Connect(
        "point.0",                                             /* id    */
        Endpoint(EndpointXy(0, 0, 0), EndpointZ(0, 0, 0, 0)),  /* start */
        ArcOffset(kCurvedRoadRadius, kCurvedRoadTheta),        /* arc   */
        EndpointZ(0, 0, 0, 0));                                /* z_end */
    Initialize(
        builder.Build(maliput::api::RoadGeometryId({"RailcarTestCurvedRoad"})));
  }

  void Initialize(std::unique_ptr<const maliput::api::RoadGeometry> road) {
    road_ = std::move(road);
    const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
    dut_.reset(new MaliputRailcar<double>(*lane));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  MaliputRailcarState<double>* continuous_state() {
    auto result = dynamic_cast<MaliputRailcarState<double>*>(
        context_->get_mutable_continuous_state_vector());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  const MaliputRailcarState<double>* state_output() const {
    auto state = dynamic_cast<const MaliputRailcarState<double>*>(
        output_->get_vector_data(dut_->state_output().get_index()));
    DRAKE_DEMAND(state != nullptr);
    return state;
  }

  const PoseVector<double>* pose_output() const {
    auto pose = dynamic_cast<const PoseVector<double>*>(
        output_->get_vector_data(dut_->pose_output().get_index()));
    DRAKE_DEMAND(pose != nullptr);
    return pose;
  }

  // Sets the configuration parameters of the vehicle.
  void SetConfig(double r, double h, double initial_speed) {
    LeafContext<double>* leaf_context =
        dynamic_cast<LeafContext<double>*>(context_.get());
    ASSERT_NE(leaf_context, nullptr);
    Parameters<double>* parameters = leaf_context->get_mutable_parameters();
    ASSERT_NE(parameters, nullptr);
    BasicVector<double>* vector_param =
        parameters->get_mutable_numeric_parameter(0);
    ASSERT_NE(vector_param, nullptr);
    MaliputRailcarConfig<double>* config =
        dynamic_cast<MaliputRailcarConfig<double>*>(vector_param);
    ASSERT_NE(config, nullptr);
    config->set_r(r);
    config->set_h(h);
    config->set_initial_speed(initial_speed);
  }

  // The arc radius and theta of the road when it is created using
  // InitializeCurvedMonoLane().
  const double kCurvedRoadRadius{10};
  const double kCurvedRoadTheta{M_PI_2};

  std::unique_ptr<const maliput::api::RoadGeometry> road_;
  std::unique_ptr<MaliputRailcar<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(MaliputRailcarTest, Topology) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());
  ASSERT_EQ(dut_->get_num_input_ports(), 0);

  ASSERT_EQ(dut_->get_num_output_ports(), 2);
  const auto& state_output = dut_->state_output();
  EXPECT_EQ(systems::kVectorValued, state_output.get_data_type());
  EXPECT_EQ(MaliputRailcarStateIndices::kNumCoordinates, state_output.size());

  const auto& pose_output = dut_->pose_output();
  EXPECT_EQ(systems::kVectorValued, pose_output.get_data_type());
  EXPECT_EQ(PoseVector<double>::kSize, pose_output.size());

  EXPECT_FALSE(dut_->HasAnyDirectFeedthrough());
}

TEST_F(MaliputRailcarTest, ZeroInitialOutput) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());
  dut_->CalcOutput(*context_, output_.get());
  auto state = state_output();
  EXPECT_EQ(state->s(), 0);
  EXPECT_EQ(state->speed(), 0);
  auto pose = pose_output();
  EXPECT_TRUE(CompareMatrices(pose->get_isometry().matrix(),
                              Eigen::Isometry3d::Identity().matrix()));
  Eigen::Translation<double, 3> translation = pose->get_translation();
  EXPECT_EQ(translation.x(), 0);
  EXPECT_EQ(translation.y(), 0);
  EXPECT_EQ(translation.z(), 0);
}

TEST_F(MaliputRailcarTest, StateAppearsInOutputDragway) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());

  continuous_state()->set_s(1.0);
  continuous_state()->set_speed(2.0);
  dut_->CalcOutput(*context_, output_.get());

  auto state = state_output();
  EXPECT_EQ(state->s(), 1);
  EXPECT_EQ(state->speed(), 2);

  auto pose = pose_output();
  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  // For the dragway, the `(s, r, h)` axes in lane-space coorespond to the
  // `(x, y, z)` axes in geo-space. In this case, `s = 1` while `r` and `h` are
  // by default zero.
  expected_pose.translation() = Eigen::Vector3d(1, 0, 0);
  EXPECT_TRUE(CompareMatrices(pose->get_isometry().matrix(),
                              expected_pose.matrix()));
}

TEST_F(MaliputRailcarTest, StateAppearsInOutputMonolane) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane());
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);

  continuous_state()->set_s(lane->length());
  continuous_state()->set_speed(3.5);
  dut_->CalcOutput(*context_, output_.get());

  ASSERT_NE(lane, nullptr);
  auto state = state_output();
  EXPECT_EQ(state->s(), lane->length());
  EXPECT_EQ(state->speed(), 3.5);

  auto pose = pose_output();
  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  {
    const Eigen::Vector3d rpy(0, 0, kCurvedRoadTheta);
    const Eigen::Vector3d xyz(kCurvedRoadRadius, kCurvedRoadRadius, 0);
    expected_pose.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }
  // The following tolerance was determined emperically.
  EXPECT_TRUE(CompareMatrices(pose->get_isometry().matrix(),
                              expected_pose.matrix(), 1e-15 /* tolerance */));
}

TEST_F(MaliputRailcarTest, NonZeroParametersAppearInOutputDragway) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());
  const double kR{1.5};
  const double kH{8.2};

  // Sets the parameters to be non-zero values.
  SetConfig(kR, kH, 1 /* initial speed */);
  dut_->CalcOutput(*context_, output_.get());
  auto pose = pose_output();
  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  expected_pose.translation() = Eigen::Vector3d(0, kR, kH);
  EXPECT_TRUE(CompareMatrices(pose->get_isometry().matrix(),
                              expected_pose.matrix()));
}

TEST_F(MaliputRailcarTest, NonZeroParametersAppearInOutputMonolane) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane());
  const double kR{1.5};
  const double kH{8.2};

  // Sets the parameters to be non-zero values.
  SetConfig(kR, kH, 1 /* initial speed */);
  dut_->CalcOutput(*context_, output_.get());
  auto start_pose = pose_output();
  Eigen::Isometry3d expected_start_pose = Eigen::Isometry3d::Identity();
  {
    const Eigen::Vector3d rpy(0, 0, 0);
    const Eigen::Vector3d xyz(0, kR, kH);
    expected_start_pose.matrix()
        << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }
  // The following tolerance was determined emperically.
  EXPECT_TRUE(CompareMatrices(start_pose->get_isometry().matrix(),
                              expected_start_pose.matrix(),
                              1e-15 /* tolerance */));

  // Moves the vehicle to be at the end of the curved lane.
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  continuous_state()->set_s(lane->length());

  dut_->CalcOutput(*context_, output_.get());
  auto end_pose = pose_output();
  Eigen::Isometry3d expected_end_pose = Eigen::Isometry3d::Identity();
  {
    const Eigen::Vector3d rpy(0, 0, M_PI_2);
    const Eigen::Vector3d xyz(kCurvedRoadRadius - kR, kCurvedRoadRadius, kH);
    expected_end_pose.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }
  // The following tolerance was determined emperically.
  EXPECT_TRUE(CompareMatrices(end_pose->get_isometry().matrix(),
                              expected_end_pose.matrix(),
                              1e-15 /* tolerance */));
}

TEST_F(MaliputRailcarTest, DerivativesDragway) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());
  // Grabs a pointer to where the EvalTimeDerivatives results end up.
  const MaliputRailcarState<double>* const result =
      dynamic_cast<const MaliputRailcarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Checks the derivatives given the default continous state with r = 0.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), MaliputRailcar<double>::kDefaultSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);  // Expect zero acceleration.

  // Checks the derivatives given a non-default continuous state with r = 0.
  continuous_state()->set_s(3.5);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), MaliputRailcar<double>::kDefaultSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);

  // Checks the derivatives given a non-default continuous state with r != 0.
  const double kSpeed{2};
  SetConfig(-2 /* r */, 0 /* h */, kSpeed);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), kSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);
}

TEST_F(MaliputRailcarTest, DerivativesMonolane) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane());
  // Grabs a pointer to where the EvalTimeDerivatives results end up.
  const MaliputRailcarState<double>* const result =
      dynamic_cast<const MaliputRailcarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);
  const double kSpeed{1};
  const double kR{1};

  // Checks the derivatives given a non-default continuous state with r != 0.
  continuous_state()->set_s(1.5);
  SetConfig(kR /* r */, 0 /* h */, kSpeed);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(),
                   kSpeed * kCurvedRoadRadius / (kCurvedRoadRadius - kR));
  EXPECT_DOUBLE_EQ(result->speed(), 0);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
