#pragma once

#include "drake/drakeCars_export.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/examples/Cars/gen/simple_car_state.h"
#include "lcmtypes/drake/lcmt_simple_car_config_t.hpp"

namespace drake {

/// SimpleCar -- model an idealized response to driving commands, neglecting
/// all physics.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to @see http://drake.mit.edu/cxx_inl.html.
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
/// * steering angle (virtual center wheel angle, with some limits);
///   a positive angle means a positive change in heading (left turn)
/// * throttle (0-1)
/// * brake (0-1)
///
/// output vector: same as state vector.
///
class DRAKECARS_EXPORT SimpleCar {
 public:
  static const drake::lcmt_simple_car_config_t kDefaultConfig;

  explicit SimpleCar(
      const drake::lcmt_simple_car_config_t& config = kDefaultConfig)
      : config_(config) {}

  const drake::lcmt_simple_car_config_t& config() const { return config_; }

  // TODO(jwnimmer-tri) Clarify the type requirements in a future PR.
  /// @name Implement the Drake System concept.
  ///
  /// @tparam ScalarType must support certain arithmetic operations,
  /// which will be specified at a future time.
  ///
  /// Instantiated templates for the following ScalarTypes are provided:
  /// - double
  /// They are already available to link against in libdrakeCars.
  ///
  /// To use other unusual ScalarType substitutions,
  /// @see http://drake.mit.edu/cxx_inl.html.
  //@{

  template <typename ScalarType>
  using StateVector = SimpleCarState<ScalarType>;
  template <typename ScalarType>
  using InputVector = DrivingCommand<ScalarType>;
  template <typename ScalarType>
  using OutputVector = SimpleCarState<ScalarType>;

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& time,
                                   const StateVector<ScalarType>& state,
                                   const InputVector<ScalarType>& input) const;

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& time,
                                  const StateVector<ScalarType>& state,
                                  const InputVector<ScalarType>& input) const;

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return false; }

  //@}

 private:
  const drake::lcmt_simple_car_config_t config_;
};

}  // namespace drake
