#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace estimators {

/**
 * A Gaussian state observer estimates the state of an observed system using its
 * input and output. All probability distributions are approximated as Gaussian.
 *
 * @system
 * name: GaussianStateObserver
 * input_ports:
 * - observed_system_input
 * - observed_system_output
 * output_ports:
 * - estimated_state
 * @endsystem
 *
 * @see ExtendedKalmanFilter()
 * @see UnscentedKalmanFilter()
 *
 * @tparam_default_scalar
 * @ingroup estimator_systems
 */
template <typename T>
class GaussianStateObserver : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GaussianStateObserver);

  /// Returns the input port that receives the observed system's input.
  virtual const InputPort<T>& get_observed_system_input_input_port() const = 0;

  /// Returns the input port that receives the observed system's output.
  virtual const InputPort<T>& get_observed_system_output_input_port() const = 0;

  /// Returns the output port that provides the estimated state.
  virtual const OutputPort<T>& get_estimated_state_output_port() const = 0;

  /// Sets the state estimate and covariance in the given @p context.
  virtual void SetStateEstimateAndCovariance(
      Context<T>* context,
      const Eigen::Ref<const Eigen::VectorX<T>>& state_estimate,
      const Eigen::Ref<const Eigen::MatrixX<T>>& state_covariance) const = 0;

  /// Gets the state estimate from the given @p context.
  virtual Eigen::VectorX<T> GetStateEstimate(
      const Context<T>& context) const = 0;

  /// Gets the state covariance from the given @p context.
  virtual Eigen::MatrixX<T> GetStateCovariance(
      const Context<T>& context) const = 0;

  virtual ~GaussianStateObserver();

 protected:
  GaussianStateObserver();

  explicit GaussianStateObserver(SystemScalarConverter converter);
};

}  // namespace estimators
}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::estimators::GaussianStateObserver);
