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

  BaseEndlessRoadCarTest() {
    const api::RBounds kLaneBounds(-2., 2.);
    const api::RBounds kDriveableBounds(-4., 4.);
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
