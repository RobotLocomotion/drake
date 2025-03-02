#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <tuple>
#include <utility>
#include <variant>

#include "drake/systems/estimators/gaussian_state_observer.h"

namespace drake {
namespace systems {
namespace estimators {

/**
 * A structure for passing optional parameters to @ref UnscentedKalmanFilter().
 */
struct UnscentedKalmanFilterOptions {
  UnscentedKalmanFilterOptions() = default;

  /**
   *  The initial state estimate. Defaults to zero initial state estimate.
   */
  std::optional<Eigen::VectorXd> initial_state_estimate{std::nullopt};

  /**
   * The initial state covariance. Defaults to ϵI where
   * `ϵ = std::numeric_limits<double>::epsilon()`.
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
   * A structure for specifying the parameters for the unscented transform.
   * The <i>m</i>ean and <i>c</i>ovariance weight for the first sigma point is
   *  @f[ W_0^{(m)} = \frac{\lambda}{n + \lambda}, \quad
   *      W_0^{(c)} = \frac{\lambda}{n + \lambda} + (1 - \alpha^2 + \beta). @f]
   * The weight for other sigma points are
   *  @f[  W_i^{(m)} = W_i^{(c)} = \frac{1}{2(n + \lambda)}. @f]
   * @f$ n @f$ is the dimension of the sigma points, and @f$ \lambda =
   * \alpha^2(n+\kappa) - n @f$.
   */
  struct UnscentedTransformParameters {
    UnscentedTransformParameters() = default;

    UnscentedTransformParameters(
        double alpha, double beta,
        std::variant<double, std::function<double(int)>> kappa);

    // Sync the member variable default values here with the python binding
    // constructor argument default values.
    /**
     * The @f$ \alpha @f$ parameter.
     */
    double alpha{1.0};

    /**
     * The @f$ \beta @f$ parameter.
     */
    double beta{2.0};

    /**
     * The @f$ \kappa @f$ parameter. Specified as either a double or a function
     * that takes @f$ n @f$ and returns a double.
     */
    std::variant<double, std::function<double(int)>> kappa{0.0};
  };

  /**
   * Parameters for the unscented transform.
   */
  UnscentedTransformParameters unscented_transform_parameters{};

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
 * Constructs an unscented Kalman filter for the given @p observed_system. The
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
 * unscented Kalman filter is synthesized.
 *
 * @param observed_system  The forward model for the observer.
 * @param observed_system_context Required because it may contain parameters
 * required to evaluate the observed system.
 * @param W The process noise covariance matrix, E[ww'].
 * @param V The measurement noise covariance matrix, E[vv'].
 * @param options Optional @ref UnscentedKalmanFilterOptions.
 * @returns The synthesized unscented Kalman filter.
 *
 * @throws std::exception if @p W is not positive definite or if @p V is
 * not positive definite.
 *
 * @ingroup estimator_systems
 */
std::unique_ptr<GaussianStateObserver<double>> UnscentedKalmanFilter(
    std::shared_ptr<const System<double>> observed_system,
    const Context<double>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V,
    const UnscentedKalmanFilterOptions& options =
        UnscentedKalmanFilterOptions());

/**
 * Same as above for constructing the unscented Kalman filter, without claiming
 * ownership of @p observed_system. The @p observed_system reference must remain
 * valid for the lifetime of the returned @p GaussianStateObserver.
 *
 * @exclude_from_pydrake_mkdoc{This overload is not bound.}
 */
std::unique_ptr<GaussianStateObserver<double>> UnscentedKalmanFilter(
    const System<double>& observed_system,
    const Context<double>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V,
    const UnscentedKalmanFilterOptions& options =
        UnscentedKalmanFilterOptions());

namespace internal {
/**
 * Check if the unscented transform parameters are ok.
 */
void CheckUnscentedTransformParams(
    int n,
    const UnscentedKalmanFilterOptions::UnscentedTransformParameters& params);

/**
 * Computes the unscented transform sigma points X and weights wₘ, Wc. Returns
 * a tuple (X, wₘ, Wc). X is a matrix where each column is a sigma point. wₘ is
 * a vector containing the mean weights, allowing us to compute the mean using
 * X * wₘ. Wc is a matrix, allowing us to compute the covariance using X * Wc *
 * X.transpose().
 */
std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::MatrixXd>
UnscentedTransform(
    const Eigen::Ref<const Eigen::VectorXd>& mean,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance,
    const UnscentedKalmanFilterOptions::UnscentedTransformParameters& params);

/**
 * A variant of the function UnscentedTransform() that takes a std::pair for
 * mean and covariance.
 */
std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::MatrixXd>
UnscentedTransform(
    const std::pair<Eigen::VectorXd, Eigen::MatrixXd>& mean_and_covariance,
    const UnscentedKalmanFilterOptions::UnscentedTransformParameters& params);

/**
 * Calculates the joint mean and covariance given two independently
 * distributed Gaussians specified by their mean and covariance.
 */
std::pair<Eigen::VectorXd, Eigen::MatrixXd> JointGaussian(
    const Eigen::Ref<const Eigen::VectorXd>& mean1,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance1,
    const Eigen::Ref<const Eigen::VectorXd>& mean2,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance2);

/**
 * Calculates the joint mean and covariance given three independently
 * distributed Gaussians specified by their mean and covariance.
 */
std::pair<Eigen::VectorXd, Eigen::MatrixXd> JointGaussian(
    const Eigen::Ref<const Eigen::VectorXd>& mean1,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance1,
    const Eigen::Ref<const Eigen::VectorXd>& mean2,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance2,
    const Eigen::Ref<const Eigen::VectorXd>& mean3,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance3);
}  // namespace internal

}  // namespace estimators
}  // namespace systems
}  // namespace drake
