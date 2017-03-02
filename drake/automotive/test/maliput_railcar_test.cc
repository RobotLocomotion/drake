#include "drake/automotive/maliput_railcar.h"

#include <cmath>
#include <limits>
#include <list>
#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/system.h"

namespace drake {

using maliput::api::LaneEnd;
using maliput::monolane::ArcOffset;
using maliput::monolane::Connection;
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

  // Creates a RoadGeometry based on monolane that consists of a straight lane
  // that's then connected to a left turning lane. See #5552 for screenshots of
  // the lane visualizations.
  //
  // @param with_s Whether the MaliputRailcar is traveling with or against the
  // lane's s-curve.
  //
  // @param flip_curve_lane Whether to flip the curved lane's s-axis. This is
  // useful for testing whether MaliputRailcar can traverse lanes that have
  // opposing s-directions.
  void InitializeTwoLaneStretchOfRoad(bool with_s = true,
                                bool flip_curve_lane = false) {
    maliput::monolane::Builder builder(
        maliput::api::RBounds(-2, 2),   /* lane_bounds       */
        maliput::api::RBounds(-4, 4),   /* driveable_bounds  */
        0.01,                           /* linear tolerance  */
        0.5 * M_PI / 180.0);            /* angular_tolerance */
    const Connection* straight_lane_connection = builder.Connect(
        "point.0",                                                /* id     */
        Endpoint(EndpointXy(0, 0, 0), EndpointZ(0, 0, 0, 0)),     /* start  */
        kStraightRoadLength,                                      /* length */
        EndpointZ(0, 0, 0, 0));                                   /* z_end  */

    if (flip_curve_lane) {
      /*
      When flip_curve_lane == true, the two lanes are configured as follows:

                  -----
                  | V |
        ----------  | |
        |>------>|<-- |
        --------------

      An OBJ rendering of this configuration is given in #5552.
      */
      const Connection* curved_lane_connection = builder.Connect(
          "point.1",                                              /* id     */
          Endpoint(                                               /* start  */
              EndpointXy(kStraightRoadLength + kCurvedRoadRadius,
                         kCurvedRoadRadius, 1.5 * M_PI),
              EndpointZ(0, 0, 0, 0)),
          ArcOffset(kCurvedRoadRadius, -kCurvedRoadTheta),        /* arc    */
          EndpointZ(0, 0, 0, 0));                                 /* z_end  */
      builder.SetDefaultBranch(
          straight_lane_connection, LaneEnd::kFinish,             /* in_end */
          curved_lane_connection, LaneEnd::kFinish);              /* out_end */
      builder.SetDefaultBranch(
          curved_lane_connection, LaneEnd::kFinish,               /* in_end */
          straight_lane_connection, LaneEnd::kFinish);            /* out_end */
    } else {
      /*
      When flip_curve_lane == false, the two lanes are configured as follows:

                  -----
                  | ^ |
        ----------  | |
        |>------>|>-- |
        --------------

      An OBJ rendering of this configuration is given in #5552.
      */
      const Connection* curved_lane_connection = builder.Connect(
          "point.1",                                              /* id     */
          Endpoint(EndpointXy(kStraightRoadLength, 0, 0),
                   EndpointZ(0, 0, 0, 0)),  /* start  */
          ArcOffset(kCurvedRoadRadius, kCurvedRoadTheta),         /* arc    */
          EndpointZ(0, 0, 0, 0));                                 /* z_end  */
      builder.SetDefaultBranch(
          straight_lane_connection, LaneEnd::kFinish,             /* in_end */
          curved_lane_connection, LaneEnd::kStart);               /* out_end */
      builder.SetDefaultBranch(
          curved_lane_connection, LaneEnd::kStart,                /* in_end */
          straight_lane_connection, LaneEnd::kFinish);            /* out_end */
    }

    std::unique_ptr<const maliput::api::RoadGeometry> road =
        builder.Build(maliput::api::RoadGeometryId(
            {"RailcarTestTwoLaneStretchOfRoad"}));
    Initialize(std::move(road), with_s);
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

  LaneDirection& lane_direction() {
    return context_->template get_mutable_abstract_state<LaneDirection>(0);
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
  void SetParams(const MaliputRailcarParams<double>& params) {
    LeafContext<double>* leaf_context =
        dynamic_cast<LeafContext<double>*>(context_.get());
    ASSERT_NE(leaf_context, nullptr);
    Parameters<double>& parameters = leaf_context->get_mutable_parameters();
    BasicVector<double>* vector_param =
        parameters.get_mutable_numeric_parameter(0);
    ASSERT_NE(vector_param, nullptr);
    MaliputRailcarParams<double>* railcar_params =
        dynamic_cast<MaliputRailcarParams<double>*>(vector_param);
    ASSERT_NE(railcar_params, nullptr);
    railcar_params->SetFrom(params);
  }

  // Obtains the lanes created by the call to InitializeTwoLaneStretchOfRoad().
  // Since a monolane::Builder was used to create the RoadGeometry, we don't
  // know which Junction contains which lane and thus need to figure it out.
  // This is done by checking the lengths of the two lanes. The straight lane
  // has a length of kStraightRoadLength = 10 while the curved lane has a length
  // of approximately 2 * PI * kCurvedRoadRadius /4 = 2 * PI * 10 / 4= 15.078.
  std::pair<const maliput::api::Lane*, const maliput::api::Lane*>
  GetStraightAndCurvedLanes() {
    DRAKE_DEMAND(road_->num_junctions() == 2);
    const maliput::api::Lane* lane_0 =
        road_->junction(0)->segment(0)->lane(0);
    const maliput::api::Lane* lane_1 =
        road_->junction(1)->segment(0)->lane(0);

    const maliput::api::Lane* straight_lane =
        (lane_0->length() == kStraightRoadLength) ? lane_0 : lane_1;
    const maliput::api::Lane* curved_lane =
        (lane_0->length() == kStraightRoadLength) ? lane_1 : lane_0;

    return std::make_pair(straight_lane, curved_lane);
  }

  // A helper method for evaluating whether MaliputRailcar can traverse a lane
  // boundary and proceed onto a default branch. The parameter `flip_curve_lane`
  // specifies whether the s-axis of the ongoing lane should be flipped.
  void DoTestLaneTraversalTest(bool flip_curve_lane) {
    EXPECT_NO_FATAL_FAILURE(InitializeTwoLaneStretchOfRoad(true /* with_s */,
                                                           flip_curve_lane));
    const maliput::api::Lane* straight_lane{};
    const maliput::api::Lane* curved_lane{};
    std::tie(straight_lane, curved_lane) = GetStraightAndCurvedLanes();

    // Verifies that the end of the straight lane is connected to:
    //  - the start of the curved lane if it is not flipped.
    //  - the end of the curved lane if it is flipped.
    std::unique_ptr<LaneEnd> straight_lane_end =
        straight_lane->GetDefaultBranch(LaneEnd::kFinish);
    ASSERT_NE(straight_lane_end, nullptr);
    EXPECT_EQ(straight_lane_end->end,
              flip_curve_lane ? LaneEnd::kFinish : LaneEnd::kStart);
    EXPECT_EQ(straight_lane_end->lane, curved_lane);

    const double kForwardSpeed(10);

    MaliputRailcarParams<double> params;
    params.set_r(1);
    params.set_h(0);
    params.set_max_speed(30);
    params.set_velocity_limit_kp(8);
    SetParams(params);

    context_->set_time(straight_lane->length() / kForwardSpeed);

    // Verifies that MaliputRailcar::DoCalcUnrestrictedUpdate() switches to the
    // start of curved lane when it is traveling forward and reaches the end of
    // the straight lane.
    continuous_state()->set_s(straight_lane->length());
    continuous_state()->set_speed(kForwardSpeed);
    lane_direction().lane = straight_lane;
    lane_direction().with_s = true;

    dut_->CalcOutput(*context_, output_.get());
    std::unique_ptr<BasicVector<double>> prior_pose = pose_output()->Clone();

    systems::DiscreteEvent<double> event;
    event.action = systems::DiscreteEvent<double>::kUnrestrictedUpdateAction;

    dut_->CalcUnrestrictedUpdate(
        *context_, event, context_->get_mutable_state());

    if (flip_curve_lane) {
      EXPECT_EQ(continuous_state()->s(), curved_lane->length());
    } else {
      EXPECT_EQ(continuous_state()->s(), 0);
    }
    EXPECT_EQ(continuous_state()->speed(), kForwardSpeed);
    EXPECT_EQ(lane_direction().with_s, !flip_curve_lane);
    EXPECT_EQ(lane_direction().lane, curved_lane);

    // Verifies the time derivative of s, s_dot, is correct. Its magnitude is
    // expected to be greater than kForwardSpeed because r = 1 and the ongoing
    // lane is curving to the left. This means the vehicle is traveling a
    // smaller radius arc relative to that of the s-axis. The smaller radius arc
    // result in the vehicle's s_dot being greater than kForwardSpeed.
    dut_->CalcTimeDerivatives(*context_, derivatives_.get());
    const MaliputRailcarState<double>* const railcar_derivatives =
      dynamic_cast<const MaliputRailcarState<double>*>(
          derivatives_->get_mutable_vector());
    if (flip_curve_lane) {
      EXPECT_LT(railcar_derivatives->s(), -kForwardSpeed);
    } else {
      EXPECT_GT(railcar_derivatives->s(), kForwardSpeed);
    }
    EXPECT_DOUBLE_EQ(railcar_derivatives->speed(), 0);

    // Verifies the vehicle's pose did not change even after switching lanes.
    dut_->CalcOutput(*context_, output_.get());
    const PoseVector<double>* post_pose = pose_output();
    // The following tolerance was empirically determined.
    EXPECT_TRUE(CompareMatrices(prior_pose->get_value(),
                                post_pose->get_value(), 1e-14 /* tolerance */));
  }

  // The length of the straight lane segment of the road when it is created
  // using InitializeTwoLaneStretchOfRoad().
  const double kStraightRoadLength{10};

  // The arc radius and theta of the road when it is created using
  // InitializeCurvedMonoLane() and InitializeTwoLaneStretchOfRoad().
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
  MaliputRailcarParams<double> params;
  params.set_r(kR);
  params.set_h(kH);
  params.set_max_speed(30);
  params.set_velocity_limit_kp(8);
  SetParams(params);
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
  MaliputRailcarParams<double> params;
  params.set_r(kR);
  params.set_h(kH);
  params.set_max_speed(30);
  params.set_velocity_limit_kp(8);
  SetParams(params);
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
  MaliputRailcarParams<double> params;
  params.set_r(-2);
  params.set_h(0);
  params.set_max_speed(kMaxSpeed);
  params.set_velocity_limit_kp(8);
  SetParams(params);
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
  MaliputRailcarParams<double> params;
  params.set_r(kR);
  params.set_h(0);
  params.set_max_speed(30);
  params.set_velocity_limit_kp(8);
  SetParams(params);
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
  MaliputRailcarParams<double> params;
  params.set_r(-2);
  params.set_h(0);
  params.set_max_speed(30);
  params.set_velocity_limit_kp(8);
  SetParams(params);
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
  MaliputRailcarParams<double> params;
  params.set_r(1);
  params.set_h(0);
  params.set_max_speed(30);
  params.set_velocity_limit_kp(8);
  SetParams(params);
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
  SetParams(params);
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
// speed is zero.
TEST_F(MaliputRailcarTest, DoCalcNextUpdateTimeWithSpeedZero) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane(true /* with_s */));
  continuous_state()->set_speed(0);
  systems::UpdateActions<double> actions;
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, std::numeric_limits<double>::infinity());

  lane_direction().with_s = false;
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, std::numeric_limits<double>::infinity());
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
  // reaches the end is MaliputRailcar<double>::kTimeEpsilon.
  continuous_state()->set_s(lane->length());
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, MaliputRailcar<double>::kTimeEpsilon);
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
  // reaches the end is MaliputRailcar<double>::kTimeEpsilon.
  continuous_state()->set_s(0);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, MaliputRailcar<double>::kTimeEpsilon);
}

// Same as the previous unit test except the road network is a curved monolane
// and with_s is true.
TEST_F(MaliputRailcarTest, DoCalcNextUpdateTimeMonolaneWithS) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane(true /* with_s */));
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed(10);

  systems::UpdateActions<double> actions;

  MaliputRailcarParams<double> params;
  params.set_r(0);
  params.set_h(0);
  params.set_max_speed(30);
  params.set_velocity_limit_kp(8);
  SetParams(params);

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
  params.set_r(1);
  SetParams(params);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_LT(actions.time, kZeroAccelerationTime);

  // Sets `r` to be negative and verifies that the time to reach the end is
  // longer.
  params.set_r(-1);
  SetParams(params);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GT(actions.time, kZeroAccelerationTime);
}

// Same as the previous unit test except with_s is false.
TEST_F(MaliputRailcarTest, DoCalcNextUpdateTimeMonolaneAgainstS) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane(false /* with_s */));
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed(10);

  systems::UpdateActions<double> actions;

  MaliputRailcarParams<double> params;
  params.set_r(0);
  params.set_h(0);
  params.set_max_speed(30);
  params.set_velocity_limit_kp(8);
  SetParams(params);

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
  params.set_r(1);
  SetParams(params);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_LT(actions.time, kZeroAccelerationTime);

  // Sets `r` to be negative and verifies that the time to reach the end is
  // longer.
  params.set_r(-1);
  SetParams(params);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GT(actions.time, kZeroAccelerationTime);
}

// Tests the two-lane stretch of road.
TEST_F(MaliputRailcarTest, TestTwoLaneStretchOfRoad) {
    EXPECT_NO_FATAL_FAILURE(InitializeTwoLaneStretchOfRoad(
        true /* with_s */, false /* flip_curve_lane */));
    const maliput::api::Lane* straight_lane{};
    const maliput::api::Lane* curved_lane{};
    std::tie(straight_lane, curved_lane) = GetStraightAndCurvedLanes();

    // Verifies that the end of the straight lane is connected to the start of
    // the curved lane if it is not flipped.
    std::unique_ptr<LaneEnd> straight_lane_start =
        straight_lane->GetDefaultBranch(LaneEnd::kStart);
    std::unique_ptr<LaneEnd> straight_lane_finish =
        straight_lane->GetDefaultBranch(LaneEnd::kFinish);
    std::unique_ptr<LaneEnd> curved_lane_start =
        curved_lane->GetDefaultBranch(LaneEnd::kStart);
    std::unique_ptr<LaneEnd> curved_lane_finish =
        curved_lane->GetDefaultBranch(LaneEnd::kFinish);
    EXPECT_EQ(straight_lane_start, nullptr);
    ASSERT_NE(straight_lane_finish, nullptr);
    EXPECT_EQ(straight_lane_finish->lane, curved_lane);
    EXPECT_EQ(straight_lane_finish->end, LaneEnd::kStart);
    ASSERT_NE(curved_lane_start, nullptr);
    EXPECT_EQ(curved_lane_finish, nullptr);
    EXPECT_EQ(curved_lane_start->lane, straight_lane);
    EXPECT_EQ(curved_lane_start->end, LaneEnd::kFinish);

    // Verifies that the end of the straight lane is connected to the end of the
    // curved lane if it is flipped.
    EXPECT_NO_FATAL_FAILURE(InitializeTwoLaneStretchOfRoad(
        true /* with_s */, true /* flip_curve_lane */));
    std::tie(straight_lane, curved_lane) = GetStraightAndCurvedLanes();
    straight_lane_start = straight_lane->GetDefaultBranch(LaneEnd::kStart);
    straight_lane_finish = straight_lane->GetDefaultBranch(LaneEnd::kFinish);
    curved_lane_start = curved_lane->GetDefaultBranch(LaneEnd::kStart);
    curved_lane_finish = curved_lane->GetDefaultBranch(LaneEnd::kFinish);
    EXPECT_EQ(straight_lane_start, nullptr);
    ASSERT_NE(straight_lane_finish, nullptr);
    EXPECT_EQ(straight_lane_finish->lane, curved_lane);
    EXPECT_EQ(straight_lane_finish->end, LaneEnd::kFinish);
    ASSERT_NE(curved_lane_finish, nullptr);
    EXPECT_EQ(curved_lane_start, nullptr);
    EXPECT_EQ(curved_lane_finish->lane, straight_lane);
    EXPECT_EQ(curved_lane_finish->end, LaneEnd::kFinish);
}

// Tests that MaliputRailcar will stop when it has reached the end of the road,
// and will not stop otherwise.
TEST_F(MaliputRailcarTest, TestStopConditions) {
  EXPECT_NO_FATAL_FAILURE(InitializeTwoLaneStretchOfRoad(
        true /* with_s */, false /* flip_curve_lane */));
  const maliput::api::Lane* straight_lane{};
  const maliput::api::Lane* curved_lane{};
  std::tie(straight_lane, curved_lane) = GetStraightAndCurvedLanes();

  const double kSpeed(10);

  MaliputRailcarParams<double> params;
  params.set_r(1);
  params.set_h(0);
  params.set_max_speed(30);
  params.set_velocity_limit_kp(8);
  SetParams(params);

  systems::DiscreteEvent<double> event;
  event.action = systems::DiscreteEvent<double>::kUnrestrictedUpdateAction;

  // Verifies that when the car is on the straight lane and is just before the
  // lane, it does not stop if it is with s, but does stop if it is against s
  // since there are no ongoing branches.
  continuous_state()->set_s(-1e-10);
  continuous_state()->set_speed(kSpeed);
  lane_direction().lane = straight_lane;
  lane_direction().with_s = true;
  dut_->CalcUnrestrictedUpdate(
      *context_, event, context_->get_mutable_state());
  EXPECT_EQ(continuous_state()->speed(), kSpeed);
  continuous_state()->set_s(-1e-10);
  lane_direction().lane = straight_lane;
  lane_direction().with_s = false;
  dut_->CalcUnrestrictedUpdate(
      *context_, event, context_->get_mutable_state());
  EXPECT_EQ(continuous_state()->speed(), 0);

  // Verifies that the car does not stop when it is on the straight lane and is
  // (1) just past the lane's start, (2) in the middle of the lane, (3) just
  // before the lane's end, or (4) just after the lane's end.
  //
  // The above should be true regardless of whether it is (1) traveling with or
  // against s or (2) traveling left or right of the s axis.
  const double epsilon = MaliputRailcar<double>::kLaneEndEpsilon;
  continuous_state()->set_speed(kSpeed);
  lane_direction().lane = straight_lane;
  for (const auto s : std::list<double>{
      1.1 * epsilon,
      straight_lane->length() / 2.0,
      straight_lane->length() - 1.1 * epsilon,
      straight_lane->length() + 1.1 * epsilon}) {
    continuous_state()->set_s(s);
    for (const auto with_s : std::list<bool>{true, false}) {
      lane_direction().with_s = with_s;
      for (const auto r : std::list<double>{-1, 0, 1}) {
        params.set_r(r);
        SetParams(params);
        dut_->CalcUnrestrictedUpdate(
            *context_, event, context_->get_mutable_state());
        EXPECT_EQ(continuous_state()->speed(), kSpeed);
      }
    }
  }

  // Verifies that the car does not stop when it is on the curved lane and is
  // (1) just before the lane's start, (2) just after the lane's start, (3) in
  // the middle of the lane, or (4) just before the lane's end.
  //
  // The above should be true regardless of whether it is (1) traveling with or
  // against s or (2) traveling left or right of the s axis.
  continuous_state()->set_speed(kSpeed);
  lane_direction().lane = curved_lane;
  for (const auto s : std::list<double>{
      -1.1 * epsilon,
      1.1 * epsilon,
      curved_lane->length() / 2.0,
      curved_lane->length() - 1.1 * epsilon}) {
    continuous_state()->set_s(s);
    for (const auto with_s : std::list<bool>{true, false}) {
      lane_direction().with_s = with_s;
      for (const auto r : std::list<double>{-1, 0, 1}) {
        params.set_r(r);
        SetParams(params);
        dut_->CalcUnrestrictedUpdate(
            *context_, event, context_->get_mutable_state());
        EXPECT_EQ(continuous_state()->speed(), kSpeed);
      }
    }
  }

  // Verifies that when the car is on the curved lane and is just after the
  // lane's end, it does not stop when it is traveling against s but stops
  // when it is traveling with s since there are no ongoing branches.
  continuous_state()->set_s(curved_lane->length() + 1e-10);
  continuous_state()->set_speed(kSpeed);
  lane_direction().lane = curved_lane;
  lane_direction().with_s = false;
  dut_->CalcUnrestrictedUpdate(*context_, event, context_->get_mutable_state());
  EXPECT_EQ(continuous_state()->speed(), kSpeed);
  lane_direction().lane = curved_lane;
  lane_direction().with_s = true;
  dut_->CalcUnrestrictedUpdate(*context_, event, context_->get_mutable_state());
  EXPECT_EQ(continuous_state()->speed(), 0);
}

// Tests the ability for a MaliputRailcar to traverse lane boundaries when the
// s-curve of the continuing lane is consistent (i.e., in the same direction)
// with the s-curve of the initial lane.
TEST_F(MaliputRailcarTest, TraverseLaneBoundaryConsistentS) {
  DoTestLaneTraversalTest(false /* flip_curve_lane */);
}

// Same as the previous test (TraverseLaneBoundaryConsistentS) except the
// s-curve of the continuing lane is inconsistent.
TEST_F(MaliputRailcarTest, TraverseLaneBoundaryInconsistentS) {
  DoTestLaneTraversalTest(true /* flip_curve_lane */);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
