#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/gen/beam_model_params.h"

namespace drake {
namespace systems {
namespace sensors {

// TODO(russt): Add support for symbolic.
/// Implements the "Beam Models of Range Finders" from section 6.3 of
///   Probabilistic Robotics (2006), by Thrun, Burgard, and Fox
///
/// This system takes a depth measurement signal as input, and outputs a noisy
/// measurement version of that signal, with some probability of returning the
/// true measurement with Gaussian noise, but also with some probability of
/// occlusions (short returns), of missed detections (returning the max depth),
/// and of returning just a (uniform) random measurement.
///
/// Four additional input ports (each of the same dimension as the depth signal)
/// are provided for the random inputs:  One for determining which of the events
/// occurred (true + noise, short return, max return, or uniform return), and
/// one
/// each for modeling the distribution of short true but noisy returns, short
/// returns, and uniform returns).
///
/// We deviate from the textbook model in one respect: both here and in the
/// textbook, the distribution over short returns and the distribution over
/// getting a noisy version of the true return (aka a "hit") are truncated.  The
/// short returns are from an exponential distribution but truncated to be less
/// than the input depth, and "hits" are drawn from a Gaussian centered at the
/// input depth but truncated at the maximum range of the sensor.  In the book,
/// these distributions are normalized so that the total probability of getting
/// a short return and/or hit stays constant (independent of the input depth).
/// Here we do not normalize, so that the probability of getting a short return
/// decreases as the input depth is smaller (there is a modeled obstacle closer
/// to the robot), and the tails of the "hit" distribution simply cause more max
/// returns as the input depth gets closer to the max range.  This was done both
/// because it is arguably a better model and because it keeps the code much
/// simpler (to allow AutoDiff and Symbolic) given the modeling framework we
/// have here that builds the output out of simple (non-truncated) random
/// variable inputs.
///
/// @tparam_nonsymbolic_scalar
/// @ingroup sensor_systems
template <typename T>
class BeamModel final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BeamModel)

  BeamModel(int num_depth_readings, double max_range);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit BeamModel(const BeamModel<U>&);

  const InputPort<T>& get_depth_input_port() const {
    return this->get_input_port(0);
  }
  const InputPort<T>& get_event_random_input_port() const {
    return this->get_input_port(1);
  }
  const InputPort<T>& get_hit_random_input_port() const {
    return this->get_input_port(2);
  }
  const InputPort<T>& get_short_random_input_port() const {
    return this->get_input_port(3);
  }
  const InputPort<T>& get_uniform_random_input_port() const {
    return this->get_input_port(4);
  }

  BeamModelParams<T>& get_mutable_parameters(Context<T>* context) const;

  double max_range() const { return max_range_; }

 private:
  void CalcOutput(const Context<T>& context, BasicVector<T>* output) const;

  const double max_range_{1.0};
};

}  // namespace sensors

// Explicitly disable symbolic::Expression (for now).
namespace scalar_conversion {
template <>
struct Traits<sensors::BeamModel> : public NonSymbolicTraits {};
}  // namespace scalar_conversion

}  // namespace systems
}  // namespace drake
