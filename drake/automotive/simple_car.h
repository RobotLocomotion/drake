#pragma once

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/drakeAutomotive_export.h"
#include "drake/systems/framework/leaf_system.h"
#include "lcmtypes/drake/lcmt_simple_car_config_t.hpp"

namespace drake {
namespace automotive {

/// SimpleCar constants that are not templated on the scalar type in use.
class DRAKEAUTOMOTIVE_EXPORT SimpleCarDefaults {
 public:
  static const drake::lcmt_simple_car_config_t kDefaultConfig;

 private:
  SimpleCarDefaults();  // disable
};

/// SimpleCar -- model an idealized response to driving commands, neglecting
/// all physics.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// configuration:
/// * see lcmt_simple_car_config_t
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
/// - drake::TaylorVarXd
///
/// They are already available to link against in libdrakeAutomotive.
///
/// To use other unusual ScalarType substitutions,
/// see http://drake.mit.edu/cxx_inl.html.
template <typename T>
class SimpleCar : public systems::LeafSystem<T> {
 public:
  explicit SimpleCar(const drake::lcmt_simple_car_config_t& config =
                         SimpleCarDefaults::kDefaultConfig);

  const drake::lcmt_simple_car_config_t& config() const { return config_; }

 public:
  // System<T> overrides
  bool has_any_direct_feedthrough() const override;
  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;
  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

 protected:
  // LeafSystem<T> overrides
  std::unique_ptr<systems::ContinuousState<T>> AllocateContinuousState()
      const override;
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;

 private:
  void DoEvalOutput(const SimpleCarState<T>&, SimpleCarState<T>*) const;
  void DoEvalTimeDerivatives(const SimpleCarState<T>&, const DrivingCommand<T>&,
                             SimpleCarState<T>*) const;

  const drake::lcmt_simple_car_config_t config_;
};

}  // namespace automotive
}  // namespace drake
