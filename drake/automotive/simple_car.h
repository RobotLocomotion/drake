#pragma once

#include <memory>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/simple_car_config.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// SimpleCar -- model an idealized response to driving commands, neglecting
/// all physics. Note that the SimpleCar can move forward, stop, turn left, and
/// turn right but *cannot* travel in reverse.
///
/// configuration:
/// * uses systems::Parameters wrapping a SimpleCarConfig
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
/// output vector: same as state vector.
///
/// @tparam T must support certain arithmetic operations;
/// for details, see ./test/simple_car_scalartype_test.cc.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
class SimpleCar : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleCar)

  SimpleCar();

  // System<T> overrides
  bool has_any_direct_feedthrough() const override;
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // LeafSystem<T> overrides
  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;

  /// Sets `config` to contain the default parameters for SimpleCar.
  static void SetDefaultParameters(SimpleCarConfig<T>* config);

 protected:
  // LeafSystem<T> overrides
  std::unique_ptr<systems::ContinuousState<T>> AllocateContinuousState()
      const override;
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::OutputPortDescriptor<T>& descriptor) const override;
  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;

 private:
  void ImplCalcOutput(const SimpleCarState<T>&, SimpleCarState<T>*) const;
  void ImplCalcTimeDerivatives(const SimpleCarConfig<T>&,
                               const SimpleCarState<T>&,
                               const DrivingCommand<T>&,
                               SimpleCarState<T>*) const;
};

}  // namespace automotive
}  // namespace drake
