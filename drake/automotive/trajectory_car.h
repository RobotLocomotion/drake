#pragma once

#include <memory>
#include <stdexcept>

#include <Eigen/Geometry>

#include "drake/automotive/curve2.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// TrajectoryCar models a car that follows a pre-established trajectory,
/// neglecting all physics.
///
/// state vector
/// * none
///
/// input vector:
/// * none
///
/// output port 0:
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
///   heading is defined around the +z axis, so positive-turn-left
/// * velocity
///   (OutputPortDescriptor getter: raw_pose_output())
///
/// output port 1: A PoseVector containing X_WC, where C is the car frame.
///   (OutputPortDescriptor getter: pose_output())
/// output port 2: A FrameVelocity containing Xdot_WC, where C is the car frame.
///   (OutputPortDescriptor getter: velocity_output())
///
/// @ingroup automotive_systems
template <typename T>
class TrajectoryCar : public systems::LeafSystem<T> {
 public:
  typedef typename Curve2<T>::Point2 Point2;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryCar)

  /// Constructs a TrajectoryCar system that traces the given @p curve,
  /// at the given constant @p speed, starting at the given @p start_time.
  /// Throws an error if the curve is empty (a zero @p path_length).
  TrajectoryCar(const Curve2<double>& curve, double speed, double start_time)
      : curve_(curve), speed_(speed), start_time_(start_time) {
    if (curve_.path_length() == 0.0) {
      throw std::invalid_argument{"empty curve"};
    }
    this->DeclareVectorOutputPort(SimpleCarState<T>());
    this->DeclareVectorOutputPort(systems::rendering::PoseVector<T>());
    this->DeclareVectorOutputPort(systems::rendering::FrameVelocity<T>());
  }

  /// See class description for details about the following ports.
  /// @{
  const systems::OutputPortDescriptor<T>& raw_pose_output() const {
    return this->get_output_port(0);
  }
  const systems::OutputPortDescriptor<T>& pose_output() const {
    return this->get_output_port(1);
  }
  const systems::OutputPortDescriptor<T>& velocity_output() const {
    return this->get_output_port(2);
  }
  /// @}

 protected:
  /// Data structure returned by CalcRawPose containing raw pose information.
  struct PositionHeading {
    Point2 position = Point2(Point2::Zero());
    T heading{0.};
  };

  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override {
    SimpleCarState<T>* const output_vector =
        dynamic_cast<SimpleCarState<T>*>(output->GetMutableVectorData(0));
    DRAKE_ASSERT(output_vector);
    ImplCalcOutput(context.get_time(), output_vector);

    systems::rendering::PoseVector<T>* const pose =
        dynamic_cast<systems::rendering::PoseVector<T>*>(
            output->GetMutableVectorData(1));
    DRAKE_ASSERT(pose);
    ImplCalcPose(context.get_time(), pose);

    systems::rendering::FrameVelocity<T>* const velocity =
      dynamic_cast<systems::rendering::FrameVelocity<T>*>(
          output->GetMutableVectorData(2));
    DRAKE_ASSERT(velocity);
    ImplCalcVelocity(context.get_time(), velocity);
  }

 private:
  void ImplCalcOutput(double time, SimpleCarState<T>* output) const {
    const auto raw_pose = CalcRawPose(time);

    // Convert raw pose to output type.
    output->set_x(raw_pose.position[0]);
    output->set_y(raw_pose.position[1]);
    output->set_heading(raw_pose.heading);
    output->set_velocity(speed_);
  }

  void ImplCalcPose(double time, systems::rendering::PoseVector<T>* pose)
      const {
    const auto raw_pose = CalcRawPose(time);

    // Convert the raw pose into a pose vector.
    pose->set_translation(Eigen::Translation<T, 3>(
        raw_pose.position[0], raw_pose.position[1], 0));
    const Vector3<T> z_axis{0.0, 0.0, 1.0};
    const Eigen::AngleAxis<T> rotation(raw_pose.heading, z_axis);
    pose->set_rotation(Eigen::Quaternion<T>(rotation));
  }

  void ImplCalcVelocity(double time,
                        systems::rendering::FrameVelocity<T>* velocity) const {
    const auto raw_pose = CalcRawPose(time);

    // Convert the state derivatives into a spatial velocity.
    multibody::SpatialVelocity<T> output;
    output.translational().x() = speed_ * std::cos(raw_pose.heading);
    output.translational().y() = speed_ * std::sin(raw_pose.heading);
    output.translational().z() = T(0);
    output.rotational().x() = T(0);
    output.rotational().y() = T(0);
    // N.B. The instantaneous rotation rate is always zero, as the Curve2
    // implmentation is based on line segments.
    output.rotational().z() = T(0);
    velocity->set_velocity(output);
  }

  const PositionHeading CalcRawPose(double time) const {
    PositionHeading result;

    // Trace the curve at a fixed speed.
    const double distance = speed_ * (time - start_time_);
    const Curve2<double>::PositionResult pose = curve_.GetPosition(distance);
    DRAKE_ASSERT(pose.position_dot.norm() > 0.0);

    result.position = pose.position;
    result.heading = std::atan2(pose.position_dot[1], pose.position_dot[0]);
    return result;
  }

  const Curve2<double> curve_;
  const double speed_{};
  const double start_time_{};
};

}  // namespace automotive
}  // namespace drake
