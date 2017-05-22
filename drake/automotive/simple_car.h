#pragma once

#include <memory>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/simple_car_params.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// SimpleCar models an idealized response to driving commands, neglecting all
/// physics. Note that SimpleCar can move forward, stop, turn left, and turn
/// right but *cannot* travel in reverse.
///
/// parameters:
/// * uses systems::Parameters wrapping a SimpleCarParams
///
/// state vector (planar for now):
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
//    heading is defined around the +z axis, so positive-turn-left
/// * velocity
///
/// input vector:
/// * steering angle (virtual center wheel angle);
///   a positive angle means a positive change in heading (left turn);
///   the value must lie within (-pi, +pi).
/// * throttle (0-1)
/// * brake (0-1)
///
/// output port 0: same as state vector.
/// output port 1: A PoseVector containing X_WC, where C is the car frame.
/// output port 2: A FrameVelocity containing Xdot_WC, where C is the car frame.
///
/// @tparam T must support certain arithmetic operations;
/// for details, see drake::symbolic::Expression.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_plants
template <typename T>
class SimpleCar : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleCar)

  SimpleCar();

  // System<T> overrides
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  const systems::OutputPort<T>& state_output() const;
  const systems::OutputPort<T>& pose_output() const;
  const systems::OutputPort<T>& velocity_output() const;

 protected:
  // System<T> overrides
  systems::System<AutoDiffXd>* DoToAutoDiffXd() const override;
  systems::System<symbolic::Expression>* DoToSymbolic() const override;

 private:
  void CalcStateOutput(const systems::Context<T>&, SimpleCarState<T>*) const;
  void CalcPose(const systems::Context<T>&,
                systems::rendering::PoseVector<T>*) const;
  void CalcVelocity(const systems::Context<T>&,
                    systems::rendering::FrameVelocity<T>*) const;

  void ImplCalcTimeDerivatives(const SimpleCarParams<T>& params,
                               const SimpleCarState<T>& state,
                               const DrivingCommand<T>& input,
                               SimpleCarState<T>* rates) const;
};

}  // namespace automotive
}  // namespace drake
