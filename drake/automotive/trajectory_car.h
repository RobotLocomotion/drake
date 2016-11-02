#pragma once

#include <stdexcept>

#include "drake/automotive/curve2.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/common/drake_export.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// A car that follows a pre-established trajectory, neglecting all physics.
///
/// state vector
/// * none
///
/// input vector:
/// * none
///
/// output vector (planar for now):
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
///   heading is defined around the +z axis, so positive-turn-left
/// * velocity
///
/// @ingroup automotive_systems
template <typename T>
class TrajectoryCar : public systems::LeafSystem<T> {
 public:
  /// Constructs a TrajectoryCar system that traces the given @p curve,
  /// at the given constant @p speed, starting at the given @p start_time.
  /// Throws an error if the curve is empty (a zero @p path_length).
  TrajectoryCar(const Curve2<double>& curve, double speed, double start_time)
      : curve_(curve), speed_(speed), start_time_(start_time) {
    if (curve_.path_length() == 0.0) {
      throw std::invalid_argument{"empty curve"};
    }
    this->DeclareOutputPort(systems::kVectorValued,
                            SimpleCarStateIndices::kNumCoordinates,
                            systems::kContinuousSampling);
  }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));
    SimpleCarState<T>* const output_vector =
        dynamic_cast<SimpleCarState<T>*>(output->GetMutableVectorData(0));
    DRAKE_ASSERT(output_vector);

    DoEvalOutput(context.get_time(), output_vector);
  }

 protected:
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override {
    return std::make_unique<SimpleCarState<T>>();
  }

 private:
  void DoEvalOutput(double time, SimpleCarState<T>* output) const {
    // Trace the curve at a fixed speed.
    const double distance = speed_ * (time - start_time_);
    const Curve2<double>::PositionResult pose = curve_.GetPosition(distance);
    DRAKE_ASSERT(pose.position_dot.norm() > 0.0);

    // Convert pose to output type.
    output->set_x(pose.position[0]);
    output->set_y(pose.position[1]);
    output->set_heading(std::atan2(pose.position_dot[1], pose.position_dot[0]));
    output->set_velocity(speed_);
  }

  const Curve2<double> curve_;
  const double speed_;
  const double start_time_;
};

}  // namespace automotive
}  // namespace drake
