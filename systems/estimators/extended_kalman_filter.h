#pragma once

#include <memory>
#include <optional>
#include <variant>

#include "drake/systems/estimators/gaussian_state_observer.h"

namespace drake {
namespace systems {
namespace estimators {

/**
 * A structure for passing optional parameters to @ref ExtendedKalmanFilter().
 */
struct ExtendedKalmanFilterOptions {
  ExtendedKalmanFilterOptions() = default;

  /**
   *  The initial state estimate. Defaults to zero initial state estimate.
   */
  std::optional<Eigen::VectorXd> initial_state_estimate{std::nullopt};

  /**
   * The initial state covariance. Defaults to zero initial state covariance.
   */
  std::optional<Eigen::MatrixXd> initial_state_covariance{std::nullopt};

  /**
   * The actuation input port of the observed system. Defaults to using the
   * first input port if it exists.
   */
  std::variant<systems::InputPortSelection, InputPortIndex>
      actuation_input_port_index{
          systems::InputPortSelection::kUseFirstInputIfItExists};

  /**
   * The measurement output port of the observed system. Defaults to using the
   * first output port.
   */
  std::variant<systems::OutputPortSelection, OutputPortIndex>
      measurement_output_port_index{
          systems::OutputPortSelection::kUseFirstOutputIfItExists};

  /**
   * The process noise input port of the observed system, which must have
   * kGaussian random type. If not specified, process noise is additive to the
   * observed system dynamics.
   */
  std::optional<InputPortIndex> process_noise_input_port_index{std::nullopt};

  /**
   * The measurement noise input port of the observed system, which must have
   * kGaussian random type. If not specified, noise is directly added to the
   * measurement output.
   */
  std::optional<InputPortIndex> measurement_noise_input_port_index{
      std::nullopt};

  /**
   * Enables the "square-root" method for computation.
   */
  bool use_square_root_method{false};

  /**
   * For observed system with continuous-time dynamics while discrete-time
   * measurements are made periodically to update the state estimation. Use this
   * parameter to specify the discrete measurement time period.
   */
  std::optional<double> discrete_measurement_time_period{std::nullopt};

  /**
   * Used in conjunction with @p discrete_measurement_time_period. Use this
   * parameter to specify the starting time of the first measurement.
   */
  double discrete_measurement_time_offset{0.0};
};

/**
 * Constructs an extended Kalman filter for the given @p observed_system. The
 * filter can be synthesized for either discrete-time or continuous-time
 * dynamics.
 *
 * **Discrete-time dynamics**
 * The observed system dynamics can be written in one of two forms:
 * x[n+1] = f(x[n], u[n]) + w[n] or x[n+1] = f(x[n], u[n], w[n]).
 * In the latter case, specify the process noise input port using
 * @p options.process_noise_input_port_index.
 * The measurement model can also be written in one of two forms:
 * y[n] = g(x[n], u[n]) + v[n] or y[n] = g(x[n], u[n], v[n]).
 * In the latter case, specify the measurement noise input port using
 * @p options.measurement_noise_input_port_index.
 *
 * **Continuous-time dynamics**
 * The observed system dynamics can be written in one of two forms:
 * ẋ = f(x, u) + w or ẋ = f(x, u, w).
 * In the latter case, specify the process noise input port using
 * @p options.process_noise_input_port_index.
 * The measurement model can also be written in one of two forms:
 * y = g(x, u) + v or y = g(x, u, v).
 * In the latter case, specify the measurement noise input port using
 * @p options.measurement_noise_input_port_index.
 * Additionally, if @p options.discrete_measurement_time_period is specified,
 * the synthesized filter will perform continuous-time process updates combined
 * with discrete-time measurement updates. Otherwise, a pure continuous-time
 * extended Kalman filter is synthesized.
 *
 * @param observed_system  The forward model for the observer.
 * @param observed_system_context Required because it may contain parameters
 * required to evaluate the observed system.
 * @param W The process noise covariance matrix, E[ww'].
 * @param V The measurement noise covariance matrix, E[vv'].
 * @param options Optional @ref ExtendedKalmanFilterOptions.
 * @returns The synthesized extended Kalman filter.
 *
 * @throws std::exception if @p W is not positive semi-definite or if @p V is
 * not positive definite.
 *
 * @ingroup estimator_systems
 * @pydrake_mkdoc_identifier{System_AutoDiffXd}
 */
std::unique_ptr<GaussianStateObserver<double>> ExtendedKalmanFilter(
    std::shared_ptr<const System<AutoDiffXd>> observed_system,
    const Context<AutoDiffXd>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V,
    const ExtendedKalmanFilterOptions& options = ExtendedKalmanFilterOptions());

/**
 * Same as above for constructing an extended Kalman filter for the given
 * @p observed_system. The  @p observed_system must be convertible to
 * System<AutoDiffXd>.
 *
 * @ingroup estimator_systems
 * @pydrake_mkdoc_identifier{System_double}
 */
std::unique_ptr<GaussianStateObserver<double>> ExtendedKalmanFilter(
    const System<double>& observed_system,
    const Context<double>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V,
    const ExtendedKalmanFilterOptions& options = ExtendedKalmanFilterOptions());

namespace internal {

/**
 * Concatenate @p vector with the vectorized @p square_matrix.
 */
template <typename T>
Eigen::VectorX<T> ConcatenateVectorAndSquareMatrix(
    const Eigen::Ref<const Eigen::VectorX<T>>& vector,
    const Eigen::Ref<const Eigen::MatrixX<T>>& square_matrix);

/**
 * Concatenate @p vector with the vectorized @p lower_tri_matrix.
 */
template <typename Derived>
Eigen::VectorX<typename Derived::Scalar> ConcatenateVectorAndLowerTriMatrix(
    const Eigen::Ref<const Eigen::VectorX<typename Derived::Scalar>>& vector,
    const Eigen::TriangularView<Derived, Eigen::Lower>& lower_tri_matrix) {
  const int size = vector.size();
  DRAKE_ASSERT(lower_tri_matrix.rows() == size);
  DRAKE_ASSERT(lower_tri_matrix.cols() == size);
  Eigen::VectorX<typename Derived::Scalar> concatenated(size +
                                                        size * (size + 1) / 2);
  concatenated.head(size) = vector;

  int idx = size;
  for (int j = 0; j < size; ++j) {
    for (int i = j; i < size; ++i) {
      concatenated(idx++) = lower_tri_matrix(i, j);
    }
  }
  return concatenated;
}

/**
 * Concatenate @p vector with the vectorized @p lower_tri_matrix.
 */
template <typename T>
Eigen::VectorX<T> ConcatenateVectorAndLowerTriMatrix(
    const Eigen::Ref<const Eigen::VectorX<T>>& vector,
    const Eigen::Ref<const Eigen::MatrixX<T>>& lower_tri_matrix);

/**
 * Extract the @p square_matrix from the @p concatenated vector returned by
 * @ref ConcatenateVectorAndSquareMatrix.
 */
template <typename T>
void ExtractSquareMatrix(
    const Eigen::Ref<const Eigen::VectorX<T>>& concatenated,
    Eigen::Ref<Eigen::MatrixX<T>> square_matrix);

/**
 * Extract the @p lower_tri_matrix from the @p concatenated vector returned by
 * @ref ConcatenateVectorAndLowerTriMatrix.
 */
template <typename T>
void ExtractLowerTriMatrix(
    const Eigen::Ref<const Eigen::VectorX<T>>& concatenated,
    Eigen::Ref<Eigen::MatrixX<T>> lower_tri_matrix);

/**
 * Check the @p observed_system for input and output ports according to
 * @p options.actuation_input_port_index,
 * @p options.measurement_output_port_index,
 * @p options.process_noise_input_port_index, and
 * @p options.measurement_noise_input_port_index. Check that the ports do not
 * repeat and have the correct types.
 *
 * @param observed_system [in] The observed system.
 * @param options [in] For port specification.
 * @param observed_system_actuation_input_port [out] Pointer to the actuation
 * input port (may be nullptr indicating no actuation input).
 * @param observed_system_measurement_output_port [out] Pointer to the
 * measurement output port (will not be nullptr).
 * @param observed_system_process_noise_input_port [out] Pointer to the
 * process noise output port (may be nullptr indicating additive noise).
 * @param observed_system_measurement_noise_input_port [out] Pointer to the
 * measurement noise output port (may be nullptr indicating additive noise).
 */
template <typename T, typename OptionsType>
void CheckObservedSystemInputOutputPorts(
    const System<T>& observed_system, const OptionsType& options,
    const InputPort<T>** observed_system_actuation_input_port,
    const OutputPort<T>** observed_system_measurement_output_port,
    const InputPort<T>** observed_system_process_noise_input_port,
    const InputPort<T>** observed_system_measurement_noise_input_port) {
  // Check observed system actuation input port.
  const InputPort<T>* actuation_input_port =
      observed_system.get_input_port_selection(
          options.actuation_input_port_index);
  if (actuation_input_port) {
    DRAKE_THROW_UNLESS(!actuation_input_port->is_random());
  }
  if (observed_system_actuation_input_port) {
    *observed_system_actuation_input_port = actuation_input_port;
  }

  // Check observed system measurement output port.
  const OutputPort<T>* measurement_output_port =
      observed_system.get_output_port_selection(
          options.measurement_output_port_index);
  DRAKE_THROW_UNLESS(measurement_output_port != nullptr);
  if (measurement_output_port->get_data_type() == kAbstractValued) {
    throw std::logic_error(
        "The specified output port is abstract-valued, but Kalman filter only "
        "supports vector-valued output ports.  Did you perhaps forget to pass "
        "a non-default `measurement_output_port_index` argument?");
  }
  DRAKE_THROW_UNLESS(measurement_output_port->size() > 0);
  if (observed_system_measurement_output_port) {
    *observed_system_measurement_output_port = measurement_output_port;
  }

  // Check observed system process noise input port.
  if (options.process_noise_input_port_index) {
    const InputPort<T>& process_noise_input_port =
        observed_system.get_input_port(*options.process_noise_input_port_index);
    if (actuation_input_port) {
      DRAKE_THROW_UNLESS(process_noise_input_port.get_index() !=
                         actuation_input_port->get_index());
    }
    DRAKE_THROW_UNLESS(process_noise_input_port.get_data_type() ==
                       kVectorValued);
    DRAKE_THROW_UNLESS(process_noise_input_port.size() > 0);
    DRAKE_THROW_UNLESS(process_noise_input_port.is_random());
    DRAKE_THROW_UNLESS(process_noise_input_port.get_random_type().value() ==
                       RandomDistribution::kGaussian);
    DRAKE_THROW_UNLESS(!observed_system.HasDirectFeedthrough(
        process_noise_input_port.get_index(),
        measurement_output_port->get_index()));
    if (observed_system_process_noise_input_port) {
      *observed_system_process_noise_input_port = &process_noise_input_port;
    }
  } else {
    if (observed_system_process_noise_input_port) {
      *observed_system_process_noise_input_port = nullptr;
    }
  }

  // Check observed system measurement noise input port.
  if (options.measurement_noise_input_port_index) {
    const InputPort<T>& measurement_noise_input_port =
        observed_system.get_input_port(
            *options.measurement_noise_input_port_index);
    if (actuation_input_port) {
      DRAKE_THROW_UNLESS(measurement_noise_input_port.get_index() !=
                         actuation_input_port->get_index());
    }
    if (options.process_noise_input_port_index) {
      DRAKE_THROW_UNLESS(measurement_noise_input_port.get_index() !=
                         *options.process_noise_input_port_index);
    }
    DRAKE_THROW_UNLESS(measurement_noise_input_port.get_data_type() ==
                       kVectorValued);
    DRAKE_THROW_UNLESS(measurement_noise_input_port.size() > 0);
    DRAKE_THROW_UNLESS(measurement_noise_input_port.is_random());
    DRAKE_THROW_UNLESS(measurement_noise_input_port.get_random_type().value() ==
                       RandomDistribution::kGaussian);
    DRAKE_THROW_UNLESS(observed_system.HasDirectFeedthrough(
        measurement_noise_input_port.get_index(),
        measurement_output_port->get_index()));
    if (observed_system_measurement_noise_input_port) {
      *observed_system_measurement_noise_input_port =
          &measurement_noise_input_port;
    }
  } else {
    if (observed_system_measurement_noise_input_port) {
      *observed_system_measurement_noise_input_port = nullptr;
    }
  }
}

}  // namespace internal

}  // namespace estimators
}  // namespace systems
}  // namespace drake
