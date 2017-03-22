#include "drake/automotive/maliput_railcar.h"

#include <cmath>
#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

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
  void InitializeDragwayLane(bool with_s = true) {
    // Defines the dragway's parameters.
    const int kNumLanes{1};
    const double kDragwayLength{50};
    const double kDragwayLaneWidth{0.5};
    const double kDragwayShoulderWidth{0.25};
    Initialize(
        std::make_unique<const maliput::dragway::RoadGeometry>(
            maliput::api::RoadGeometryId({"RailcarTestDragway"}), kNumLanes,
            kDragwayLength, kDragwayLaneWidth, kDragwayShoulderWidth), with_s);
  }

  void InitializeCurvedMonoLane(bool with_s = true) {
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
        builder.Build(maliput::api::RoadGeometryId({"RailcarTestCurvedRoad"})),
        with_s);
  }

  void InitializeSlopedCurvedMonoLane(bool with_s = true) {
    maliput::monolane::Builder builder(
        maliput::api::RBounds(-2, 2),   /* lane_bounds       */
        maliput::api::RBounds(-4, 4),   /* driveable_bounds  */
        0.01,                           /* linear tolerance  */
        0.5 * M_PI / 180.0);            /* angular_tolerance */
    builder.Connect(
        "point.0",                                             /* id    */
        Endpoint(EndpointXy(0, 0, 0), EndpointZ(0, 0, 0, 0)),  /* start */
        ArcOffset(kCurvedRoadRadius, kCurvedRoadTheta),        /* arc   */
        EndpointZ(2, 0, 0.5, 0));                              /* z_end */
    Initialize(
        builder.Build(maliput::api::RoadGeometryId({"RailcarTestCurvedRoad"})),
        with_s);
  }

  void Initialize(std::unique_ptr<const maliput::api::RoadGeometry> road,
      bool with_s) {
    road_ = std::move(road);
    const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
    dut_.reset(new MaliputRailcar<double>(LaneDirection(lane, with_s)));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  void SetInputValue(double desired_acceleration) {
    DRAKE_DEMAND(dut_ != nullptr);
    DRAKE_DEMAND(context_ != nullptr);
    context_->FixInputPort(dut_->command_input().get_index(),
        BasicVector<double>::Make(desired_acceleration));
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

  // Sets the configuration parameters of the railcar.
  void SetConfig(const MaliputRailcarConfig<double>& config) {
    LeafContext<double>* leaf_context =
        dynamic_cast<LeafContext<double>*>(context_.get());
    ASSERT_NE(leaf_context, nullptr);
    Parameters<double>& parameters = leaf_context->get_mutable_parameters();
    BasicVector<double>* vector_param =
        parameters.get_mutable_numeric_parameter(0);
    ASSERT_NE(vector_param, nullptr);
    MaliputRailcarConfig<double>* railcar_config =
        dynamic_cast<MaliputRailcarConfig<double>*>(vector_param);
    ASSERT_NE(railcar_config, nullptr);
    railcar_config->SetFrom(config);
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
  ASSERT_EQ(dut_->get_num_input_ports(), 1);

  ASSERT_EQ(dut_->get_num_output_ports(), 3);
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
  EXPECT_EQ(state->s(), MaliputRailcar<double>::kDefaultInitialS);
  EXPECT_EQ(state->speed(), MaliputRailcar<double>::kDefaultInitialSpeed);
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
  const double kS{1};
  const double kSpeed{2};

  continuous_state()->set_s(kS);
  continuous_state()->set_speed(kSpeed);
  dut_->CalcOutput(*context_, output_.get());

  auto state = state_output();
  EXPECT_EQ(state->s(), kS);
  EXPECT_EQ(state->speed(), kSpeed);

  auto pose = pose_output();
  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  // For the dragway, the `(s, r, h)` axes in lane-space coorespond to the
  // `(x, y, z)` axes in geo-space. In this case, `s = kS` while `r` and `h` are
  // by default zero.
  expected_pose.translation() = Eigen::Vector3d(kS, 0, 0);
  EXPECT_TRUE(CompareMatrices(pose->get_isometry().matrix(),
                              expected_pose.matrix()));
}

TEST_F(MaliputRailcarTest, StateAppearsInOutputMonolane) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane());
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed{3.5};

  continuous_state()->set_s(lane->length());
  continuous_state()->set_speed(kSpeed);
  dut_->CalcOutput(*context_, output_.get());

  ASSERT_NE(lane, nullptr);
  auto state = state_output();
  EXPECT_EQ(state->s(), lane->length());
  EXPECT_EQ(state->speed(), kSpeed);

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
  MaliputRailcarConfig<double> config;
  config.set_r(kR);
  config.set_h(kH);
  config.set_initial_speed(1);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);
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
  MaliputRailcarConfig<double> config;
  config.set_r(kR);
  config.set_h(kH);
  config.set_initial_speed(1);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);
  dut_->CalcOutput(*context_, output_.get());
  auto start_pose = pose_output();
  Eigen::Isometry3d expected_start_pose = Eigen::Isometry3d::Identity();
  {
    const Eigen::Vector3d rpy(0, 0, 0);
    const Eigen::Vector3d xyz(0, kR, kH);
    expected_start_pose.matrix()
        << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }
  // The following tolerance was determined empirically.
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
  // The following tolerance was determined empirically.
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

  // Sets the input command.
  SetInputValue(0 /* desired_acceleration */);

  // Checks the derivatives given the default continuous state with r = 0.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), MaliputRailcar<double>::kDefaultInitialSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);  // Expect zero acceleration.

  // Checks that the acceleration is zero given an acceleration command of zero
  // and a non-default continuous state with r = 0.
  continuous_state()->set_s(3.5);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), MaliputRailcar<double>::kDefaultInitialSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);

  // Checks that the acceleration is zero given an acceleration command of zero
  // and a non-default continuous state with r != 0.
  const double kS{1.5};
  const double kSlowSpeed{2};
  const double kMaxSpeed{30};
  MaliputRailcarConfig<double> config;
  config.set_r(-2);
  config.set_h(0);
  config.set_initial_speed(kSlowSpeed);
  config.set_max_speed(kMaxSpeed);
  config.set_velocity_limit_kp(8);
  SetConfig(config);
  continuous_state()->set_s(kS);
  continuous_state()->set_speed(kSlowSpeed);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), kSlowSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);

  // Checks that a positive acceleration command of `kPosAccelCmd` results in
  // the actual acceleration being equal to the commanded acceleration when the
  // vehicle's current speed of `kSlowSpeed` is far lower than the vehicle's
  // maximum speed of `kMaxSpeed`.
  const double kPosAccelCmd{5};
  SetInputValue(kPosAccelCmd);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), kSlowSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), kPosAccelCmd);

  // Checks that a negative acceleration command of `kNegAccelCmd` results in
  // the actual acceleration being equal to the commanded acceleration when the
  // vehicle's current speed of `kFastSpeed` is far higher than 0.
  const double kFastSpeed{27};
  const double kNegAccelCmd{-1};
  SetInputValue(kNegAccelCmd);
  continuous_state()->set_speed(kFastSpeed);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), kFastSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), kNegAccelCmd);
}

TEST_F(MaliputRailcarTest, DerivativesMonolane) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane());
  // Grabs a pointer to where the EvalTimeDerivatives results end up.
  const MaliputRailcarState<double>* const result =
      dynamic_cast<const MaliputRailcarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Sets the input command.
  SetInputValue(0  /* desired acceleration */);

  const double kInitialSpeed{1};
  const double kR{1};

  // Checks the derivatives given a non-default continuous state with r != 0.
  continuous_state()->set_s(1.5);
  MaliputRailcarConfig<double> config;
  config.set_r(kR);
  config.set_h(0);
  config.set_initial_speed(kInitialSpeed);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(),
      kInitialSpeed * kCurvedRoadRadius / (kCurvedRoadRadius - kR));
  EXPECT_DOUBLE_EQ(result->speed(), 0);
}

// Tests that connecting the input port is optional, i.e., that it can contain
// a nullptr value.
TEST_F(MaliputRailcarTest, InputPortNotConnected) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());
  // Grabs a pointer to where the EvalTimeDerivatives results end up.
  const MaliputRailcarState<double>* const result =
      dynamic_cast<const MaliputRailcarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  ASSERT_NO_FATAL_FAILURE(
      dut_->CalcTimeDerivatives(*context_, derivatives_.get()));
  EXPECT_DOUBLE_EQ(result->s(), MaliputRailcar<double>::kDefaultInitialSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);

  // Checks that the derivatives contain the speed set by the configuration and
  // zero acceleration.
  const double kS{2.25};
  const double kInitialSpeed{2};
  MaliputRailcarConfig<double> config;
  config.set_r(-2);
  config.set_h(0);
  config.set_initial_speed(kInitialSpeed);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);
  continuous_state()->set_s(kS);
  continuous_state()->set_speed(kInitialSpeed);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), kInitialSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);
}

// Tests the correctness of the derivatives when the vehicle travels in the
// decreasing `s` direction.
TEST_F(MaliputRailcarTest, DecreasingSDragway) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane(false /* with_s */));
  // Grabs a pointer to where the EvalTimeDerivatives results end up.
  const MaliputRailcarState<double>* const result =
      dynamic_cast<const MaliputRailcarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Sets the input command.
  const double kAccelCmd{1.5};  // The acceleration command.
  SetInputValue(kAccelCmd);

  // Checks that the time derivative of `s` is the negative of the speed, which
  // happens when the vehicle is traveling in the negative `s` direction.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), -MaliputRailcar<double>::kDefaultInitialSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), kAccelCmd);

  // Verifies that the vehicle's pose is correct. Since the vehicle is traveling
  // against s, its orientation is expected to be flipped.
  dut_->CalcOutput(*context_, output_.get());
  const PoseVector<double>* pose = pose_output();
  PoseVector<double> expected_pose;
  expected_pose.set_translation({0, 0, 0});
  expected_pose.set_rotation({0, 0, 0, -1});
  // The following tolerance was determined empirically.
  EXPECT_TRUE(CompareMatrices(
      pose->get_value(), expected_pose.get_value(), 1e-15 /* tolerance */));
}

TEST_F(MaliputRailcarTest, DecreasingSMonolane) {
  EXPECT_NO_FATAL_FAILURE(InitializeSlopedCurvedMonoLane(false /* with_s */));
  // Sets the r != 0 and s != 0.
  const double kS{2.25};
  const double kInitialSpeed{2};
  MaliputRailcarConfig<double> config;
  config.set_r(1);
  config.set_h(0);
  config.set_initial_speed(kInitialSpeed);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);
  continuous_state()->set_s(kS);
  continuous_state()->set_speed(kInitialSpeed);

  // Obtains the against-s pose.
  dut_->CalcOutput(*context_, output_.get());
  const PoseVector<double>* against_s_pose = pose_output();
  Eigen::Translation<double, 3> against_s_translation =
      against_s_pose->get_translation();
  Eigen::Quaternion<double> against_s_orientation =
      against_s_pose->get_rotation();

  // Obtains the with-s pose.
  EXPECT_NO_FATAL_FAILURE(InitializeSlopedCurvedMonoLane(true /* with_s */));
  SetConfig(config);
  continuous_state()->set_s(kS);
  continuous_state()->set_speed(kInitialSpeed);
  dut_->CalcOutput(*context_, output_.get());
  const PoseVector<double>* with_s_pose = pose_output();

  Eigen::Translation<double, 3> with_s_translation =
      with_s_pose->get_translation();
  Eigen::Quaternion<double> with_s_orientation = with_s_pose->get_rotation();

  // Verifies that the with-s and against-s translations are equal.
  EXPECT_TRUE(with_s_translation.isApprox(against_s_translation));

  // Verifies that the with-s and against-s orientations are opposite of
  // each other. This is done by checking that the dot product of the two
  // quaternions is equal to zero. The following tolerance was empirically
  // determined.
  EXPECT_NEAR(with_s_orientation.dot(against_s_orientation), 0, 1e-15);
}

// Tests the correctness of MaliputRailcar::DoCalcNextUpdateTime() when the
// road network is a dragway and with_s is true.
TEST_F(MaliputRailcarTest, DoCalcNextUpdateTimeDragwayWithS) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane(true /* with_s */));
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed(10);

  systems::UpdateActions<double> actions;

  // Computes the time to reach the end of the lane assuming a speed of kSpeed.
  const double kZeroAccelerationTime = lane->length() / kSpeed;

  // Verifies that the time till reaching the end of the lane is equal to the
  // lane length divided by the vehicle's speed.
  context_->set_time(0);
  continuous_state()->set_s(0);
  continuous_state()->set_speed(kSpeed);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime);
  EXPECT_EQ(actions.events.at(0).action,
            systems::DiscreteEvent<double>::kUnrestrictedUpdateAction);

  // Verifies that when the vehicle is mid-way through the lane, the time till
  // it reaches the end is cut in half.
  continuous_state()->set_s(lane->length() / 2);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GE(actions.time, 0);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime / 2);

  // Verifies that when the vehicle is at the end of the lane, the time till it
  // reaches the end is zero.
  continuous_state()->set_s(lane->length());
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, 0);
}

// Same as the previous unit test except with_s is false.
TEST_F(MaliputRailcarTest, DoCalcNextUpdateTimeDragwayAgainstS) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane(false /* with_s */));
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed(10);

  systems::UpdateActions<double> actions;

  // Computes the time to reach the end of the lane assuming a speed of kSpeed
  // and zero acceleration.
  const double kZeroAccelerationTime = lane->length() / kSpeed;

  // Verifies that the time till reaching the end of the lane is equal to the
  // lane length divided by the vehicle's speed.
  context_->set_time(0);
  continuous_state()->set_s(lane->length());
  continuous_state()->set_speed(kSpeed);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GE(actions.time, 0);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime);

  // Verifies that when the vehicle is mid-way through the lane, the time till
  // it reaches the end is cut in half.
  continuous_state()->set_s(lane->length() / 2);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GE(actions.time, 0);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime / 2);

  // Verifies that when the vehicle is at the end of the lane, the time till it
  // reaches the end is zero.
  continuous_state()->set_s(0);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, 0);
}

// Same as the previous unit test except the road network is a curved monolane
// and with_s is true.
TEST_F(MaliputRailcarTest, DoCalcNextUpdateTimeMonolaneWithS) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane(true /* with_s */));
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed(10);

  systems::UpdateActions<double> actions;

  MaliputRailcarConfig<double> config;
  config.set_r(0);
  config.set_h(0);
  config.set_initial_speed(kSpeed);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);

  // Computes the time to reach the end of the lane assuming a speed of kSpeed
  // and zero acceleration.
  const double kZeroAccelerationTime = lane->length() / kSpeed;

  // Verifies that the time till reaching the end of the lane is equal to the
  // lane length divided by the vehicle's speed.
  context_->set_time(0);
  continuous_state()->set_s(0);
  continuous_state()->set_speed(kSpeed);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime);

  // Sets `r` to be positive and verifies that the time to reach the end is
  // shorter.
  config.set_r(1);
  SetConfig(config);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_LT(actions.time, kZeroAccelerationTime);

  // Sets `r` to be negative and verifies that the time to reach the end is
  // longer.
  config.set_r(-1);
  SetConfig(config);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GT(actions.time, kZeroAccelerationTime);
}

// Same as the previous unit test except with_s is false.
TEST_F(MaliputRailcarTest, DoCalcNextUpdateTimeMonolaneAgainstS) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane(false /* with_s */));
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed(10);

  systems::UpdateActions<double> actions;

  MaliputRailcarConfig<double> config;
  config.set_r(0);
  config.set_h(0);
  config.set_initial_speed(kSpeed);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);

  // Computes the time to reach the end of the lane assuming a speed of kSpeed
  // and zero acceleration.
  const double kZeroAccelerationTime = lane->length() / kSpeed;

  // Verifies that the time till reaching the end of the lane is equal to the
  // lane length divided by the vehicle's speed.
  context_->set_time(0);
  continuous_state()->set_s(lane->length());
  continuous_state()->set_speed(kSpeed);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime);

  // Sets `r` to be positive and verifies that the time to reach the end is
  // shorter.
  config.set_r(1);
  SetConfig(config);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_LT(actions.time, kZeroAccelerationTime);

  // Sets `r` to be negative and verifies that the time to reach the end is
  // longer.
  config.set_r(-1);
  SetConfig(config);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GT(actions.time, kZeroAccelerationTime);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
