#include "drake/automotive/endless_road_car.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"

namespace drake {
namespace automotive {
namespace {

namespace api = drake::maliput::api;
namespace mono = drake::maliput::monolane;
namespace utility = drake::maliput::utility;

// EndlessRoadCar is basically three related vehicle/controller types,
// configured at runtime.  All of them operate on an InfiniteCircuitRoad.

class BaseEndlessRoadCarTest : public ::testing::Test {
 protected:
  const double kLinearTolerance {1e-6};
  const double kAngularTolerance {1e-6 * M_PI};
  const double kRingRadius {100.};
  const api::RBounds kDriveableBounds {-4., 4.};

  BaseEndlessRoadCarTest() {
    const api::RBounds kLaneBounds(-2., 2.);
    mono::Builder b(kLaneBounds, kDriveableBounds,
                    kLinearTolerance, kAngularTolerance);
    {
      // A simple ring road.
      const mono::EndpointZ kFlatZ(0., 0., 0., 0.);
      const mono::Endpoint start {{kRingRadius, 0., M_PI / 2.}, kFlatZ};

      b.Connect("0", start, mono::ArcOffset(kRingRadius, M_PI * 2.), kFlatZ);
    }
    source_road_ = b.Build({"source"});

    infinite_road_ = std::make_unique<utility::InfiniteCircuitRoad>(
        api::RoadGeometryId {"infinite"},
        source_road_.get(),
        api::LaneEnd(source_road_->junction(0)->segment(0)->lane(0),
                     api::LaneEnd::kStart),
        std::vector<const api::Lane*>());
  }

  std::unique_ptr<const api::RoadGeometry> source_road_;
  std::unique_ptr<const utility::InfiniteCircuitRoad> infinite_road_;
};



// Type #1:  'kNone' control_type == no inputs
class NoneEndlessRoadCarTest : public BaseEndlessRoadCarTest {
 protected:
  NoneEndlessRoadCarTest() {
    //////////    BaseEndlessRoadCarTest::SetUp();
    dut_ = std::make_unique<EndlessRoadCar<double>>(
        "dut", infinite_road_.get(), EndlessRoadCar<double>::kNone);

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
    /////    SetInputValue(0, 0, 0);
  }

/////  void SetInputValue(double steering_angle, double throttle, double brake) {
/////    auto value = std::make_unique<DrivingCommand<double>>();
/////    value->set_steering_angle(steering_angle);
/////    value->set_throttle(throttle);
/////    value->set_brake(brake);
/////    context_->SetInputPort(
/////        0, std::make_unique<systems::FreestandingInputPort>(std::move(value)));
/////  }

  EndlessRoadCarState<double>* continuous_state() {
    auto result = dynamic_cast<EndlessRoadCarState<double>*>(
        context_->get_mutable_continuous_state_vector());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(NoneEndlessRoadCarTest, Topology) {
  ASSERT_EQ(0, dut_->get_num_input_ports());
  ////  const auto& input_descriptor = dut_->get_input_port(0);
  ////  EXPECT_EQ(systems::kVectorValued, input_descriptor.get_data_type());
  ////  EXPECT_EQ(DrivingCommandIndices::kNumCoordinates, input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(EndlessRoadCarStateIndices::kNumCoordinates,
            output_descriptor.size());
}

TEST_F(NoneEndlessRoadCarTest, Output) {
  // Grab a pointer to where the CalcOutput results end up.
  const EndlessRoadCarState<double>* const result =
      dynamic_cast<const EndlessRoadCarState<double>*>(
          output_->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Starting state and output is all zeros.
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->speed());

  // New state just propagates through.
  continuous_state()->set_s(1.0);
  continuous_state()->set_r(2.0);
  continuous_state()->set_heading(3.0);
  continuous_state()->set_speed(4.0);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->s());
  EXPECT_EQ(2.0, result->r());
  EXPECT_EQ(3.0, result->heading());
  EXPECT_EQ(4.0, result->speed());

  // However, per the TODO in ImplCalcOutput, we clamp negative speeds.
  // (I.e., braking decelerations should cause us to stop, but not go
  // backwards.)
  continuous_state()->set_speed(-0.0006);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(0., result->speed());

/////  // The input doesn't matter.
/////  SetInputValue(0.3, 0.5, 0.7);
/////  dut_->CalcOutput(*context_, output_.get());
/////  EXPECT_EQ(1.0, result->x());
/////  EXPECT_EQ(2.0, result->y());
/////  EXPECT_EQ(3.0, result->heading());
/////  EXPECT_EQ(4.0, result->velocity());
}

TEST_F(NoneEndlessRoadCarTest, Derivatives) {
  ///////  const double kTolerance = 1e-10;

  // Grab a pointer to where the EvalTimeDerivatives results end up.
  const EndlessRoadCarState<double>* const result =
      dynamic_cast<const EndlessRoadCarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Starting state is all zeros.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->speed());

  // There's no input; derivatives should just be zero.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->speed());

/////  // Half throttle yields half of the max acceleration.
/////  const double max_acceleration =
/////      EndlessRoadCar<double>::get_default_config().max_acceleration();
/////  SetInputValue(0.0, 0.5, 0.0);
/////  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
/////  EXPECT_EQ(0.0, result->x());
/////  EXPECT_EQ(0.0, result->y());
/////  EXPECT_EQ(0.0, result->heading());
/////  EXPECT_NEAR(0.5 * max_acceleration, result->velocity(), kTolerance);
/////
/////  // Set speed to mid-range, with zero input.
/////  continuous_state()->set_velocity(10.0);
/////  SetInputValue(0.0, 0.0, 0.0);
/////  // At heading 0, we are moving along +x.
/////  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
/////  EXPECT_NEAR(10.0, result->x(), kTolerance);
/////  EXPECT_EQ(0.0, result->y());
/////  EXPECT_EQ(0.0, result->heading());
/////  EXPECT_EQ(0.0, result->velocity());
/////
/////  // A non-zero steering_angle turns in the same direction.  We'd like to turn
/////  // at 0.1 rad/s at a speed of 10m/s, so we want a curvature of 0.01.
/////  const double wheelbase = EndlessRoadCar<double>::get_default_config().wheelbase();
/////  const double steering_angle = std::atan(0.01 * wheelbase);
/////  SetInputValue(steering_angle, 0.0, 0.0);
/////  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
/////  EXPECT_NEAR(10.0, result->x(), kTolerance);
/////  EXPECT_EQ(0.0, result->y());
/////  EXPECT_NEAR(0.1, result->heading(), kTolerance);
/////  EXPECT_EQ(0.0, result->velocity());
/////  SetInputValue(-steering_angle, 0.0, 0.0);
/////  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
/////  EXPECT_NEAR(10.0, result->x(), kTolerance);
/////  EXPECT_EQ(0.0, result->y());
/////  EXPECT_NEAR(-0.1, result->heading(), kTolerance);
/////  EXPECT_EQ(0.0, result->velocity());
/////
/////  // Half brake yields half of the max acceleration.
/////  SetInputValue(0.0, 0.0, 0.5);
/////  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
/////  EXPECT_NEAR(10.0, result->x(), kTolerance);
/////  EXPECT_EQ(0.0, result->y());
/////  EXPECT_EQ(0.0, result->heading());
/////  EXPECT_NEAR(-0.5 * EndlessRoadCar<double>::get_default_config().max_acceleration(),
/////              result->velocity(), kTolerance);
/////
/////  // A heading of +90deg points us at +y.
/////  continuous_state()->set_heading(0.5 * M_PI);
/////  SetInputValue(0.0, 0.0, 0.0);
/////  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
/////  EXPECT_NEAR(0.0, result->x(), kTolerance);
/////  EXPECT_NEAR(10.0, result->y(), kTolerance);
/////  EXPECT_EQ(0.0, result->heading());
/////  EXPECT_EQ(0.0, result->velocity());
}


// Type #2:  'kUser' control_type == explicit DrivingCommand input
class UserEndlessRoadCarTest : public BaseEndlessRoadCarTest {
 protected:
  UserEndlessRoadCarTest() {
    dut_ = std::make_unique<EndlessRoadCar<double>>(
        "dut", infinite_road_.get(), EndlessRoadCar<double>::kUser);

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
    SetInputValue(0, 0, 0);
  }

  void SetInputValue(double steering_angle, double throttle, double brake) {
    auto value = std::make_unique<DrivingCommand<double>>();
    value->set_steering_angle(steering_angle);
    value->set_throttle(throttle);
    value->set_brake(brake);
    context_->SetInputPort(
        0, std::make_unique<systems::FreestandingInputPort>(std::move(value)));
  }

  EndlessRoadCarState<double>* continuous_state() {
    auto result = dynamic_cast<EndlessRoadCarState<double>*>(
        context_->get_mutable_continuous_state_vector());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};


TEST_F(UserEndlessRoadCarTest, Topology) {
  ASSERT_EQ(1, dut_->get_num_input_ports());
  const auto& input_descriptor = dut_->get_input_port(0);
  EXPECT_EQ(systems::kVectorValued, input_descriptor.get_data_type());
  EXPECT_EQ(DrivingCommandIndices::kNumCoordinates, input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(EndlessRoadCarStateIndices::kNumCoordinates,
            output_descriptor.size());
}


TEST_F(UserEndlessRoadCarTest, Output) {
  // Grab a pointer to where the CalcOutput results end up.
  const EndlessRoadCarState<double>* const result =
      dynamic_cast<const EndlessRoadCarState<double>*>(
          output_->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Starting state and output is all zeros.
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->speed());

  // New state just propagates through.
  continuous_state()->set_s(1.0);
  continuous_state()->set_r(2.0);
  continuous_state()->set_heading(3.0);
  continuous_state()->set_speed(4.0);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->s());
  EXPECT_EQ(2.0, result->r());
  EXPECT_EQ(3.0, result->heading());
  EXPECT_EQ(4.0, result->speed());

  // The input doesn't matter.
  SetInputValue(0.3, 0.5, 0.7);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->s());
  EXPECT_EQ(2.0, result->r());
  EXPECT_EQ(3.0, result->heading());
  EXPECT_EQ(4.0, result->speed());

  // However, per the TODO in ImplCalcOutput, we clamp negative speeds.
  // (I.e., braking decelerations should cause us to stop, but not go
  // backwards.)
  continuous_state()->set_speed(-0.0006);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(0., result->speed());
}


TEST_F(UserEndlessRoadCarTest, Derivatives) {
  const double kTolerance = 1e-10;

  // Grab a pointer to where the EvalTimeDerivatives results end up.
  const EndlessRoadCarState<double>* const result =
      dynamic_cast<const EndlessRoadCarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Starting state is all zeros.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->speed());

  // Half throttle yields half of the max acceleration.
  const double max_acceleration =
      EndlessRoadCar<double>::get_default_config().max_acceleration();
  SetInputValue(0.0, 0.5, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(0.5 * max_acceleration, result->speed(), kTolerance);

  // Set speed to mid-range, with zero input.
  continuous_state()->set_speed(10.0);
  SetInputValue(0.0, 0.0, 0.0);
  // At heading 0, we are moving along +s.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->s(), kTolerance);
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->speed());

  // A non-zero steering_angle turns in the same direction.  We'd like to turn
  // at 0.1 rad/s at a speed of 10m/s, so we want a curvature of 0.01.
  const double wheelbase =
      EndlessRoadCar<double>::get_default_config().wheelbase();
  const double steering_angle = std::atan(0.01 * wheelbase);
  SetInputValue(steering_angle, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->s(), kTolerance);
  EXPECT_EQ(0.0, result->r());
  EXPECT_NEAR(0.1, result->heading(), kTolerance);
  EXPECT_EQ(0.0, result->speed());
  SetInputValue(-steering_angle, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->s(), kTolerance);
  EXPECT_EQ(0.0, result->r());
  EXPECT_NEAR(-0.1, result->heading(), kTolerance);
  EXPECT_EQ(0.0, result->speed());

  // Half brake yields half of the max deceleration.
  const double max_deceleration =
      EndlessRoadCar<double>::get_default_config().max_deceleration();
  SetInputValue(0.0, 0.0, 0.5);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->s(), kTolerance);
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(-0.5 * max_deceleration, result->speed(), kTolerance);

  // A heading of +90deg points us at +r.
  continuous_state()->set_heading(0.5 * M_PI);
  SetInputValue(0.0, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(0.0, result->s(), kTolerance);
  EXPECT_NEAR(10.0, result->r(), kTolerance);
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->speed());

  // The "Magic Guard Rail" should clamp derivatives which send us beyond
  // the driveable road boundary.
  SetInputValue(0.0, 0.0, 0.0);
  // (Recall:  continuous_state()->speed() is still 10.)

  // ...test at maximum-r boundary:
  continuous_state()->set_r(kDriveableBounds.r_max);
  continuous_state()->set_heading(M_PI / 6.); // 30 degrees
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0 * std::sqrt(3.) / 2. *
              kRingRadius / (kRingRadius - kDriveableBounds.r_max),
              result->s(), kTolerance);
  EXPECT_NEAR(0., result->r(), kTolerance); // clamped
  continuous_state()->set_heading(-M_PI / 6.); // -30 degrees
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0 * std::sqrt(3.) / 2. *
              kRingRadius / (kRingRadius - kDriveableBounds.r_max),
              result->s(), kTolerance);
  EXPECT_NEAR(-10.0 * 0.5, result->r(), kTolerance);

  // ...test at minimum-r boundary:
  continuous_state()->set_r(kDriveableBounds.r_min);
  continuous_state()->set_heading(M_PI / 6.); // 30 degrees
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0 * std::sqrt(3.) / 2. *
              kRingRadius / (kRingRadius - kDriveableBounds.r_min),
              result->s(), kTolerance);
  EXPECT_NEAR(10.0 * 0.5, result->r(), kTolerance);
  continuous_state()->set_heading(-M_PI / 6.); // -30 degrees
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0 * std::sqrt(3.) / 2. *
              kRingRadius / (kRingRadius - kDriveableBounds.r_min),
              result->s(), kTolerance);
  EXPECT_NEAR(0., result->r(), kTolerance); // clamped
}


// Type #3:  'kIdm' control_type == EndlessRoadOracleOutput feeding IDM
class IdmEndlessRoadCarTest : public BaseEndlessRoadCarTest {
 protected:
  IdmEndlessRoadCarTest() {
    dut_ = std::make_unique<EndlessRoadCar<double>>(
        "dut", infinite_road_.get(), EndlessRoadCar<double>::kIdm);

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
    SetInputValue(0., 0.);
  }

  void SetInputValue(double net_delta_sigma, double delta_sigma_dot) {
    auto value = std::make_unique<EndlessRoadOracleOutput<double>>();
    value->set_net_delta_sigma(net_delta_sigma);
    value->set_delta_sigma_dot(delta_sigma_dot);
    context_->SetInputPort(
        0, std::make_unique<systems::FreestandingInputPort>(std::move(value)));
  }

  EndlessRoadCarState<double>* continuous_state() {
    auto result = dynamic_cast<EndlessRoadCarState<double>*>(
        context_->get_mutable_continuous_state_vector());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};


TEST_F(IdmEndlessRoadCarTest, Topology) {
  ASSERT_EQ(1, dut_->get_num_input_ports());
  const auto& input_descriptor = dut_->get_input_port(0);
  EXPECT_EQ(systems::kVectorValued, input_descriptor.get_data_type());
  EXPECT_EQ(EndlessRoadOracleOutputIndices::kNumCoordinates,
            input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(EndlessRoadCarStateIndices::kNumCoordinates,
            output_descriptor.size());
}


TEST_F(IdmEndlessRoadCarTest, Output) {
  // Grab a pointer to where the CalcOutput results end up.
  const EndlessRoadCarState<double>* const result =
      dynamic_cast<const EndlessRoadCarState<double>*>(
          output_->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Starting state and output is all zeros.
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->speed());

  // New state just propagates through.
  continuous_state()->set_s(1.0);
  continuous_state()->set_r(2.0);
  continuous_state()->set_heading(3.0);
  continuous_state()->set_speed(4.0);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->s());
  EXPECT_EQ(2.0, result->r());
  EXPECT_EQ(3.0, result->heading());
  EXPECT_EQ(4.0, result->speed());

  // The input doesn't matter.
  SetInputValue(0.3, 0.5);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->s());
  EXPECT_EQ(2.0, result->r());
  EXPECT_EQ(3.0, result->heading());
  EXPECT_EQ(4.0, result->speed());

  // However, per the TODO in ImplCalcOutput, we clamp negative speeds.
  // (I.e., braking decelerations should cause us to stop, but not go
  // backwards.)
  continuous_state()->set_speed(-0.0006);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(0., result->speed());
}


TEST_F(IdmEndlessRoadCarTest, Derivatives) {
  const double kTolerance = 1e-10;

  // Grab a pointer to where the EvalTimeDerivatives results end up.
  const EndlessRoadCarState<double>* const result =
      dynamic_cast<const EndlessRoadCarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Starting state is all zeros.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->speed());

  // Accelerations are determined by IDM parameters hard-coded into
  // EndlessRoadCar::ComputeIdmAccelerations().  We'll exercise a few
  // state/input configurations.
  //
  // 1)  Current velocity matches IDM desired, and distance is enormous.
  //     Longitudinal acceleration should be zero.
  continuous_state()->set_speed(30.);
  SetInputValue(1e26, 0.);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(30.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(0.0, result->speed(), kTolerance);
  // 2)  Current velocity matches IDM desired, but distance is tiny.
  //     Longitudinal acceleration should be clamped to max deceleration.
  const double max_deceleration =
      EndlessRoadCar<double>::get_default_config().max_deceleration();
  continuous_state()->set_speed(30.);
  SetInputValue(1e-5, 0.);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(30.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(-max_deceleration, result->speed());
  // 3)  Current velocity is zero, and distance is enormous.
  //     Longitudinal acceleration should be max acceleration.
  const double max_acceleration =
      EndlessRoadCar<double>::get_default_config().max_acceleration();
  continuous_state()->set_speed(0.);
  SetInputValue(1e26, 0.);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(max_acceleration, result->speed(), kTolerance);
  // 3)  Current velocity matches IDM desired, but distance is minimal
  ///    and differential-velocity is large
  //     Longitudinal acceleration should be negative.
  continuous_state()->set_speed(30.);
  SetInputValue(2.0, 100.);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(30.0, result->s());
  EXPECT_EQ(0.0, result->r());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_GT(0.0, result->speed());
}













#if 0

class EndlessRoadCarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new EndlessRoadCar<double>);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
    SetInputValue(0, 0, 0);
  }

  void SetInputValue(double steering_angle, double throttle, double brake) {
    auto value = std::make_unique<DrivingCommand<double>>();
    value->set_steering_angle(steering_angle);
    value->set_throttle(throttle);
    value->set_brake(brake);
    context_->SetInputPort(
        0, std::make_unique<systems::FreestandingInputPort>(std::move(value)));
  }

  EndlessRoadCarState<double>* continuous_state() {
    auto result = dynamic_cast<EndlessRoadCarState<double>*>(
        context_->get_mutable_continuous_state_vector());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(EndlessRoadCarTest, Topology) {
  ASSERT_EQ(1, dut_->get_num_input_ports());
  const auto& input_descriptor = dut_->get_input_port(0);
  EXPECT_EQ(systems::kVectorValued, input_descriptor.get_data_type());
  EXPECT_EQ(DrivingCommandIndices::kNumCoordinates, input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(EndlessRoadCarStateIndices::kNumCoordinates, output_descriptor.size());
}

TEST_F(EndlessRoadCarTest, Output) {
  // Grab a pointer to where the CalcOutput results end up.
  const EndlessRoadCarState<double>* const result =
      dynamic_cast<const EndlessRoadCarState<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Starting state and output is all zeros.
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());

  // New state just propagates through.
  continuous_state()->set_x(1.0);
  continuous_state()->set_y(2.0);
  continuous_state()->set_heading(3.0);
  continuous_state()->set_velocity(4.0);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->x());
  EXPECT_EQ(2.0, result->y());
  EXPECT_EQ(3.0, result->heading());
  EXPECT_EQ(4.0, result->velocity());

  // The input doesn't matter.
  SetInputValue(0.3, 0.5, 0.7);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->x());
  EXPECT_EQ(2.0, result->y());
  EXPECT_EQ(3.0, result->heading());
  EXPECT_EQ(4.0, result->velocity());
}

TEST_F(EndlessRoadCarTest, Derivatives) {
  const double kTolerance = 1e-10;

  // Grab a pointer to where the EvalTimeDerivatives results end up.
  const EndlessRoadCarState<double>* const result =
      dynamic_cast<const EndlessRoadCarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Starting state is all zeros.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());

  // Half throttle yields half of the max acceleration.
  const double max_acceleration =
      EndlessRoadCar<double>::get_default_config().max_acceleration();
  SetInputValue(0.0, 0.5, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(0.5 * max_acceleration, result->velocity(), kTolerance);

  // Set speed to mid-range, with zero input.
  continuous_state()->set_velocity(10.0);
  SetInputValue(0.0, 0.0, 0.0);
  // At heading 0, we are moving along +x.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());

  // A non-zero steering_angle turns in the same direction.  We'd like to turn
  // at 0.1 rad/s at a speed of 10m/s, so we want a curvature of 0.01.
  const double wheelbase = EndlessRoadCar<double>::get_default_config().wheelbase();
  const double steering_angle = std::atan(0.01 * wheelbase);
  SetInputValue(steering_angle, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_NEAR(0.1, result->heading(), kTolerance);
  EXPECT_EQ(0.0, result->velocity());
  SetInputValue(-steering_angle, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_NEAR(-0.1, result->heading(), kTolerance);
  EXPECT_EQ(0.0, result->velocity());

  // Half brake yields half of the max acceleration.
  SetInputValue(0.0, 0.0, 0.5);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_NEAR(-0.5 * EndlessRoadCar<double>::get_default_config().max_acceleration(),
              result->velocity(), kTolerance);

  // A heading of +90deg points us at +y.
  continuous_state()->set_heading(0.5 * M_PI);
  SetInputValue(0.0, 0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(0.0, result->x(), kTolerance);
  EXPECT_NEAR(10.0, result->y(), kTolerance);
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());
}
#endif


}  // namespace
}  // namespace automotive
}  // namespace drake
