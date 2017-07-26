#pragma once

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/automotive/calc_smooth_acceleration.h"
#include "drake/automotive/curve2.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/gen/trajectory_car_params.h"
#include "drake/automotive/gen/trajectory_car_state.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/extract_double.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_base.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// TrajectoryCar models a car that follows a pre-established trajectory.  Note
/// that TrajectoryCar can move forward (up to a given "soft" speed limit) but
/// cannot travel in reverse.
///
/// parameters:
/// * uses systems::Parameters wrapping a TrajectoryCarParams
///
/// state vector:
/// * A TrajectoryCarState, consisting of a position and speed along the given
///   curve, provided as the constructor parameter.
///
/// input vector:
/// * desired acceleration, a systems::BasicVector of size 1 (optional input).
///   If left unconnected, the trajectory car will travel at a constant speed
///   specified in the TrajectoryCarState.
///
/// output port 0:
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
///   heading is defined around the +z axis, so positive-turn-left
/// * velocity
///   (OutputPort getter: raw_pose_output())
///
/// output port 1: A PoseVector containing X_WC, where C is the car frame.
///   (OutputPort getter: pose_output())
///
/// output port 2: A FrameVelocity containing Xdot_WC, where C is the car frame.
///   (OutputPort getter: velocity_output())
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_plants
template <typename T>
class TrajectoryCar : public systems::LeafSystem<T> {
 public:
  typedef typename Curve2<double>::Point2 Point2d;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryCar)

  /// Constructs a TrajectoryCar system that traces a given two-dimensional @p
  /// curve.  Throws an error if the curve is empty (has a zero @p path_length).
  explicit TrajectoryCar(Curve2<double> curve)
      : curve_(std::move(curve)) {
    if (curve_.path_length() == 0.0) {
      throw std::invalid_argument{"empty curve"};
    }
    this->DeclareInputPort(systems::kVectorValued, 1 /* single-valued input */);
    this->DeclareVectorOutputPort(&TrajectoryCar::CalcStateOutput);
    this->DeclareVectorOutputPort(&TrajectoryCar::CalcPoseOutput);
    this->DeclareVectorOutputPort(&TrajectoryCar::CalcVelocityOutput);
    this->DeclareContinuousState(TrajectoryCarState<T>());
    this->DeclareNumericParameter(TrajectoryCarParams<T>());
  }

  /// The command input port (optional).
  const systems::InputPortDescriptor<T>& command_input() const {
    return this->get_input_port(0);
  }
  /// See class description for details about the following ports.
  /// @{
  const systems::OutputPort<T>& raw_pose_output() const {
    return this->get_output_port(0);
  }
  const systems::OutputPort<T>& pose_output() const {
    return this->get_output_port(1);
  }
  const systems::OutputPort<T>& velocity_output() const {
    return this->get_output_port(2);
  }
  /// @}

 protected:
  /// Data structure returned by CalcRawPose containing raw pose information.
  struct PositionHeading {
    Point2d position = Point2d(Point2d::Zero());
    double heading{0.};
  };

  void CalcStateOutput(const systems::Context<T>& context,
                       SimpleCarState<T>* output_vector) const {
    const TrajectoryCarState<T>& state = GetState(context);
    const auto raw_pose = CalcRawPose(state);
    ImplCalcOutput(raw_pose, state, output_vector);
  }

  void CalcPoseOutput(const systems::Context<T>& context,
                      systems::rendering::PoseVector<T>* pose) const {
    const auto raw_pose = CalcRawPose(GetState(context));
    ImplCalcPose(raw_pose, pose);
  }

  void CalcVelocityOutput(
      const systems::Context<T>& context,
      systems::rendering::FrameVelocity<T>* velocity) const {
    const TrajectoryCarState<T>& state = GetState(context);
    const auto raw_pose = CalcRawPose(state);
    ImplCalcVelocity(raw_pose, state, velocity);
  }

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override {
    // Obtain the parameters.
    const TrajectoryCarParams<T>& params =
        this->template GetNumericParameter<TrajectoryCarParams>(context, 0);

    // Obtain the state.
    const TrajectoryCarState<T>* const state =
        dynamic_cast<const TrajectoryCarState<T>*>(
            &context.get_continuous_state_vector());
    DRAKE_ASSERT(state);

    // Obtain the input.
    const systems::BasicVector<T>* input =
        this->template EvalVectorInput<systems::BasicVector>(context, 0);

    // If the input is null, then apply a default acceleration of zero.
    const auto default_input = systems::BasicVector<T>::Make(0.);
    if (input == nullptr) {
      input = default_input.get();
    }
    DRAKE_ASSERT(input->size() == 1);  // Expect the input to have only a single
                                       // acceleration value.

    // Obtain the result structure.
    DRAKE_ASSERT(derivatives != nullptr);
    systems::VectorBase<T>* const vector_derivatives =
        derivatives->get_mutable_vector();
    DRAKE_ASSERT(vector_derivatives);
    TrajectoryCarState<T>* const rates =
        dynamic_cast<TrajectoryCarState<T>*>(vector_derivatives);
    DRAKE_ASSERT(rates);

    ImplCalcTimeDerivatives(params, *state, *input, rates);
  }

 private:
  void ImplCalcOutput(const PositionHeading& raw_pose,
                      const TrajectoryCarState<T>& state,
                      SimpleCarState<T>* output) const {
    // Convert raw pose to output type.
    output->set_x(raw_pose.position[0]);
    output->set_y(raw_pose.position[1]);
    output->set_heading(raw_pose.heading);
    output->set_velocity(state.speed());
  }

  void ImplCalcPose(const PositionHeading& raw_pose,
                    systems::rendering::PoseVector<T>* pose) const {
    // Convert the raw pose into a pose vector.
    pose->set_translation(Eigen::Translation<T, 3>(raw_pose.position[0],
                                                   raw_pose.position[1], 0));
    const Vector3<T> z_axis{0.0, 0.0, 1.0};
    const Eigen::AngleAxis<T> rotation(raw_pose.heading, z_axis);
    pose->set_rotation(Eigen::Quaternion<T>(rotation));
  }

  void ImplCalcVelocity(const PositionHeading& raw_pose,
                        const TrajectoryCarState<T>& state,
                        systems::rendering::FrameVelocity<T>* velocity) const {
    using std::cos;
    using std::sin;

    // Convert the state derivatives into a spatial velocity.
    multibody::SpatialVelocity<T> output;
    output.translational().x() = state.speed() * cos(raw_pose.heading);
    output.translational().y() = state.speed() * sin(raw_pose.heading);
    output.translational().z() = T(0);
    output.rotational().x() = T(0);
    output.rotational().y() = T(0);
    // N.B. The instantaneous rotation rate is always zero, as the Curve2 is
    // constructed from line segments.
    output.rotational().z() = T(0);
    velocity->set_velocity(output);
  }

  void ImplCalcTimeDerivatives(const TrajectoryCarParams<T>& params,
                               const TrajectoryCarState<T>& state,
                               const systems::BasicVector<T>& input,
                               TrajectoryCarState<T>* rates) const {
    using std::max;

    // Create an acceleration profile that caps the maximum speed of the vehicle
    // as it approaches or exceeds the `params.max_speed()` limit, passing the
    // input acceleration through when away from the limit.  Note that
    // accelerations of zero are passed through unaffected.
    const T desired_acceleration = input.GetAtIndex(0);
    const T smooth_acceleration =
        calc_smooth_acceleration(desired_acceleration, params.max_speed(),
                                 params.speed_limit_kp(), state.speed());

    // Don't allow small negative velocities to affect position.
    const T nonneg_velocity = max(T(0), state.speed());

    rates->set_position(nonneg_velocity);
    rates->set_speed(smooth_acceleration);
  }

  // Extract the appropriately-typed state from the context.
  const TrajectoryCarState<T>& GetState(
      const systems::Context<T>& context) const {
    auto state = dynamic_cast<const TrajectoryCarState<T>*>(
        &context.get_continuous_state_vector());
    DRAKE_DEMAND(state != nullptr);
    return *state;
  }

  // Computes the PositionHeading of the trajectory car based on the car's
  // current position along the curve.
  const PositionHeading CalcRawPose(const TrajectoryCarState<T>& state) const {
    using std::atan2;

    PositionHeading result;

    // Compute the curve at the current longitudinal (along-curve) position.
    const typename Curve2<double>::PositionResult pose =
        curve_.GetPosition(ExtractDoubleOrThrow(state.position()));
    // TODO(jadecastro): Now that the curve is a function of position rather
    // than time, we are not acting on a `trajectory` anymore.  Rename this
    // System to PathFollowingCar or something similar.
    DRAKE_ASSERT(pose.position_dot.norm() > 0.0);

    result.position = pose.position;
    result.heading = atan2(pose.position_dot[1], pose.position_dot[0]);
    return result;
  }

  TrajectoryCar<AutoDiffXd>* DoToAutoDiffXd() const override {
    return new TrajectoryCar<AutoDiffXd>(curve_);
  }

  const Curve2<double> curve_;
};

}  // namespace automotive
}  // namespace drake
