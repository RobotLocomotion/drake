#include "drake/automotive/trajectory_car.h"

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/curve2.h"
#include "drake/automotive/test/autodiff_test_utilities.h"
#include "drake/common/extract_double.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace automotive {
namespace {

typedef Curve2<double> Curve2d;
typedef Curve2d::Point2 Point2d;

using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;
using test::CheckDerivatives;
using test::SetDerivatives;

template <typename T>
void SetAcceleration(const TrajectoryCar<T>& car_dut,
                     const T& acceleration_input,
                     systems::Context<T>* context) {
  context->FixInputPort(car_dut.command_input().get_index(),
                        systems::BasicVector<T>::Make(acceleration_input));
}

// Empty curves are rejected.
GTEST_TEST(TrajectoryCarTest, StationaryTest) {
  const std::vector<Point2d> empty_waypoints{};
  const Curve2d empty_curve{empty_waypoints};
  EXPECT_THROW((TrajectoryCar<double>{empty_curve}), std::exception);
}

template <typename T>
class TrajectoryCarTest : public ::testing::Test {};

typedef ::testing::Types<double, AutoDiffXd> Implementations;
TYPED_TEST_CASE(TrajectoryCarTest, Implementations);

// Check the car's progress along some simple paths.  We just want to test our
// ImplCalcFoo() methods.  We can assume that Curve2 is correct, because it has
// its own unit test.
TYPED_TEST(TrajectoryCarTest, ConstantSpeedTest) {
  using T = TypeParam;

  struct Case {
    double heading;
    double distance;
    double speed;
    double start_position;
  };
  const std::vector<Case> cases{
      // Various headings.
      {-0.999 * M_PI, 1.0, 1.0, 0.0},
      {-0.875 * M_PI, 1.0, 1.0, 0.0},
      {-0.500 * M_PI, 1.0, 1.0, 0.0},
      {-0.125 * M_PI, 1.0, 1.0, 0.0},
      {0.000 * M_PI, 1.0, 1.0, 0.0},
      {0.125 * M_PI, 1.0, 1.0, 0.0},
      {0.500 * M_PI, 1.0, 1.0, 0.0},
      {0.875 * M_PI, 1.0, 1.0, 0.0},
      {0.999 * M_PI, 1.0, 1.0, 0.0},

      // Various distances / speeds.
      {0.125 * M_PI, 10.0, 1.0, 0.0},
      {0.125 * M_PI, 1.0, 10.0, 0.0},
      {0.125 * M_PI, 1.0, 0.1, 0.0},

      // Various starting positions.
      {0.125 * M_PI, 1.0, 1.0, 100.0},
      {0.125 * M_PI, 1.0, 1.0, 10.0},
      {0.125 * M_PI, 1.0, 1.0, -10.0},
      {0.125 * M_PI, 1.0, 1.0, -100.0},
  };

  for (const auto& it : cases) {
    // Choose an arbitrary start point, then add just one segment
    // leaving the start point, based on the parameters in the test case.
    const Point2d start{20.0, 30.0};
    const Point2d heading_vector{std::cos(it.heading), std::sin(it.heading)};
    const std::vector<Point2d> waypoints{
        start,  // BR
        start + (heading_vector * it.distance),
    };
    const Curve2d curve{waypoints};

    // The "device under test".
    const TrajectoryCar<T> car_dut{curve};

    // Check that the systems' outputs are correct over the entire duration of
    // the trajectory.
    const double total_distance = it.distance;
    const double finish_position = it.start_position + total_distance;
    const double start_time = it.start_position / it.speed;
    const double end_time = finish_position / it.speed;

    systems::Simulator<T> simulator(car_dut);
    systems::Context<T>& context = simulator.get_mutable_context();
    std::unique_ptr<systems::SystemOutput<T>> all_output =
        car_dut.AllocateOutput(context);

    // Specify the initial position and speed.
    auto car_state = dynamic_cast<TrajectoryCarState<T>*>(
        &context.get_mutable_continuous_state_vector());
    ASSERT_NE(nullptr, car_state);

    // Set a one-dimensional derivative vector taken with respect to
    // start_position.
    T start_position(it.start_position);
    T speed(it.speed);
    SetDerivatives(&start_position, Vector1d{1.});

    car_state->set_position(start_position);
    car_state->set_speed(speed);  // This speed will be held constant throughout
                                  // a run of the simulator.
    simulator.Initialize();

    for (double time = start_time; time <= end_time; time += 0.1) {
      simulator.StepTo(time - start_time);

      const double fractional_progress =
          std::min(std::max(0.0, (time * it.speed) / total_distance), 1.0);
      const Point2d expected_position =
          start + (heading_vector * it.distance * fractional_progress);
      const double kMaxErrorPos = 1e-6;
      const double kMaxErrorRad = 1e-6;

      car_dut.CalcOutput(context, all_output.get());

      ASSERT_EQ(3, all_output->get_num_ports());

      // Tests the raw pose output.
      const SimpleCarState<T>* raw_pose =
          dynamic_cast<const SimpleCarState<T>*>(
              all_output->get_vector_data(
                  car_dut.raw_pose_output().get_index()));
      ASSERT_NE(nullptr, raw_pose);
      EXPECT_EQ(SimpleCarStateIndices::kNumCoordinates, raw_pose->size());

      // N.B. We tolerate some small integration errors.
      EXPECT_NEAR(expected_position(0), ExtractDoubleOrThrow(raw_pose->x()),
                  kMaxErrorPos);
      EXPECT_NEAR(expected_position(1), ExtractDoubleOrThrow(raw_pose->y()),
                  kMaxErrorPos);
      EXPECT_NEAR(it.heading, ExtractDoubleOrThrow(raw_pose->heading()),
                  kMaxErrorRad);
      EXPECT_DOUBLE_EQ(it.speed, ExtractDoubleOrThrow(raw_pose->velocity()));

      // The AutoDiff derivatives of position are nonzero except at the ends.
      if (car_state->position() <= 0. ||
          car_state->position() > total_distance) {
        CheckDerivatives(raw_pose->x(), Vector1d{0.});
        CheckDerivatives(raw_pose->y(), Vector1d{0.});
      } else {
        CheckDerivatives(raw_pose->x(), Vector1d{std::cos(it.heading)});
        CheckDerivatives(raw_pose->y(), Vector1d{std::sin(it.heading)});
      }
      CheckDerivatives(raw_pose->heading(), Vector1d{0.});
      CheckDerivatives(raw_pose->velocity(), Vector1d{0.});

      // Tests the PoseVector output.
      const PoseVector<T>* pose = dynamic_cast<const PoseVector<T>*>(
          all_output->get_vector_data(car_dut.pose_output().get_index()));
      ASSERT_NE(nullptr, pose);
      EXPECT_EQ(PoseVector<double>::kSize, pose->size());

      // N.B. We tolerate some small integration errors.
      EXPECT_NEAR(expected_position(0),
                  ExtractDoubleOrThrow(
                      pose->get_translation().translation().x()),
                  kMaxErrorPos);
      EXPECT_NEAR(expected_position(1),
                  ExtractDoubleOrThrow(
                      pose->get_translation().translation().y()),
                  kMaxErrorPos);
      EXPECT_NEAR(std::cos(it.heading / 2),
                  ExtractDoubleOrThrow(pose->get_rotation().w()),
                  kMaxErrorRad);
      EXPECT_NEAR(std::sin(it.heading / 2),
                  ExtractDoubleOrThrow(pose->get_rotation().z()),
                  kMaxErrorRad);

      // The AutoDiff derivatives of position are nonzero except at the ends.
      if (car_state->position() <= 0. ||
          car_state->position() > total_distance) {
        CheckDerivatives(pose->get_translation().translation().x(),
                         Vector1d{0.});
        CheckDerivatives(pose->get_translation().translation().y(),
                         Vector1d{0.});
      } else {
        CheckDerivatives(pose->get_translation().translation().x(),
                         Vector1d{std::cos(it.heading)});
        CheckDerivatives(pose->get_translation().translation().y(),
                         Vector1d{std::sin(it.heading)});
      }
      CheckDerivatives(pose->get_rotation().w(), Vector1d{0.});
      CheckDerivatives(pose->get_rotation().z(), Vector1d{0.});

      // Tests the FrameVelocity output.
      const FrameVelocity<T>* velocity =
          dynamic_cast<const FrameVelocity<T>*>(
              all_output->get_vector_data(
                  car_dut.velocity_output().get_index()));
      ASSERT_NE(nullptr, velocity);
      EXPECT_EQ(FrameVelocity<T>::kSize, velocity->size());

      EXPECT_NEAR(it.speed * cos(it.heading),
                  ExtractDoubleOrThrow(
                      velocity->get_velocity().translational().x()),
                  kMaxErrorRad);
      EXPECT_NEAR(it.speed * sin(it.heading),
                  ExtractDoubleOrThrow(
                      velocity->get_velocity().translational().y()),
                  kMaxErrorRad);

      // The AutoDiff derivatives are zero.
      CheckDerivatives(velocity->get_velocity().translational().x(),
                       Vector1d{0.});
      CheckDerivatives(velocity->get_velocity().translational().y(),
                       Vector1d{0.});
    }
  }
}

// Tests the derivatives when a non-zero acceleration is provided as an input.
TYPED_TEST(TrajectoryCarTest, AccelerationInputTest) {
  using std::abs;
  using T = TypeParam;

  T kInitialPathPosition = 0.;
  T kInitialSpeed = 20.;

  const std::vector<Point2d> waypoints{{10., 20.},  // BR
                                       {10., 30.}};
  const Curve2d curve{waypoints};
  const TrajectoryCar<T> dut{curve};  // The device under test.

  auto context = dut.CreateDefaultContext();
  auto derivatives = dut.AllocateTimeDerivatives();
  auto car_derivatives = dynamic_cast<const TrajectoryCarState<T>*>(
      &derivatives->get_mutable_vector());
  ASSERT_NE(nullptr, car_derivatives);
  const TrajectoryCarParams<T> default_params;

  // Set the initial position and speed.
  auto car_state = dynamic_cast<TrajectoryCarState<T>*>(
      &context->get_mutable_continuous_state_vector());
  ASSERT_NE(nullptr, car_state);
  car_state->set_position(kInitialPathPosition);
  car_state->set_speed(kInitialSpeed);

  // Create some non-saturating acceleration command.
  T kAccelerationInput = 1.5;

  // Set a one-dimensional derivative vector taken with respect to
  // input acceleration.
  SetDerivatives(&kAccelerationInput, Vector1d{1.});

  EXPECT_LT(kAccelerationInput,
            default_params.speed_limit_kp() * abs(kInitialSpeed));
  SetAcceleration(dut, kAccelerationInput, context.get());
  dut.CalcTimeDerivatives(*context, derivatives.get());

  // Tests the derivatives.
  EXPECT_EQ(kInitialSpeed, car_derivatives->position());
  EXPECT_EQ(kAccelerationInput, car_derivatives->speed());

  // Recover the seeded AutoDiff derivative on input acceleration as the
  // AutoDiff derivative of the car's true acceleration.
  CheckDerivatives(car_derivatives->position(), Vector1d{0.});
  CheckDerivatives(car_derivatives->speed(), Vector1d{1.});
}

// Tests the derivatives when the car's speed is beyond the specified saturation
// bounds.
TYPED_TEST(TrajectoryCarTest, SaturatingSpeedTest) {
  using T = TypeParam;

  T kInitialPathPosition = 0.;

  const std::vector<Point2d> waypoints{{10., 20.},  // BR
                                       {10., 30.}};
  const Curve2d curve{waypoints};
  const TrajectoryCar<T> dut{curve};  // The device under test.

  auto context = dut.CreateDefaultContext();
  auto derivatives = dut.AllocateTimeDerivatives();
  auto car_derivatives = dynamic_cast<const TrajectoryCarState<T>*>(
      &derivatives->get_mutable_vector());
  ASSERT_NE(nullptr, car_derivatives);
  const TrajectoryCarParams<T> default_params;

  // Set the initial position and speed.
  auto car_state = dynamic_cast<TrajectoryCarState<T>*>(
      &context->get_mutable_continuous_state_vector());
  ASSERT_NE(nullptr, car_state);
  car_state->set_position(kInitialPathPosition);

  // Set the speed to some positive saturated value.
  T kOverSpeed = default_params.max_speed() + 5.;

  // Use a zero acceleration and set its derivative.
  T kZeroAcceleration(0.);
  SetDerivatives(&kZeroAcceleration, Vector1d{1.});

  car_state->set_speed(kOverSpeed);
  SetAcceleration(dut, kZeroAcceleration, context.get());
  dut.CalcTimeDerivatives(*context, derivatives.get());

  // Tests that the derivatives do in fact reflect that the speed has been
  // saturated.
  EXPECT_EQ(kOverSpeed, car_derivatives->position());
  EXPECT_GE(0., car_derivatives->speed());  // Exceed upper limit => decelerate.

  // Check that the AutoDiff derivatives survive.
  CheckDerivatives(car_derivatives->position(), Vector1d{0.});
  CheckDerivatives(car_derivatives->speed(), Vector1d{0.});

  // Set the speed to some negative value.
  T kUnderSpeed = -5.;

  car_state->set_speed(kUnderSpeed);
  SetAcceleration(dut, kZeroAcceleration, context.get());
  dut.CalcTimeDerivatives(*context, derivatives.get());

  // Tests that the derivatives saturate negative speeds at zero.
  EXPECT_EQ(0., car_derivatives->position());
  EXPECT_LE(0., car_derivatives->speed());  // Exceed lower limit => accelerate.

  // Check that the AutoDiff derivatives survive.
  CheckDerivatives(car_derivatives->position(), Vector1d{0.});
  CheckDerivatives(car_derivatives->speed(), Vector1d{0.});
}

GTEST_TEST(TrajectoryCarTest, ToAutoDiff) {
  const std::vector<Point2d> waypoints{{10., 20.},  // BR
                                       {10., 30.}};
  const Curve2d curve{waypoints};
  const TrajectoryCar<double> dut{curve};

  EXPECT_TRUE(is_autodiffxd_convertible(dut, [&](const auto& autodiff_dut) {
    auto context = autodiff_dut.CreateDefaultContext();
    auto output = autodiff_dut.AllocateOutput(*context);
    auto derivatives = autodiff_dut.AllocateTimeDerivatives();

    context->FixInputPort(
        dut.command_input().get_index(),
        systems::BasicVector<AutoDiffXd>::Make(AutoDiffXd(0.)));

    // Check that the public methods can be called without exceptions.
    autodiff_dut.CalcOutput(*context, output.get());
    autodiff_dut.CalcTimeDerivatives(*context, derivatives.get());
  }));
}

GTEST_TEST(TrajectoryCarTest, ToSymbolic) {
  const std::vector<Point2d> waypoints{{10., 20.},  // BR
                                       {10., 30.}};
  const Curve2d curve{waypoints};
  const TrajectoryCar<double> dut{curve};

  // We do not support symbolic form.
  EXPECT_EQ(dut.ToSymbolicMaybe(), nullptr);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
