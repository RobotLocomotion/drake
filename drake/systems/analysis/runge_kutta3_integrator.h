#pragma once

#include <utility>

#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A third-order Runge Kutta integrator with a third order error estimate.
 * @tparam T A double or autodiff type.
 *
 * This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
 * this class, please refer to http://drake.mit.edu/cxx_inl.html.
 *
 * Instantiated templates for the following kinds of T's are provided:
 * - double
 *
 * For a discussion of this Runge-Kutta method, see [Butcher, 1987]. The
 * embedded error estimate was derived using the method mentioned in
 * [Hairer, 1993].
 *
 * The Butcher tableaux for this integrator follows:
 * <pre>
 *        |
 * 0      |
 * 1/2    | 1/2
 * 1      | -1          2
 * ---------------------------------------------------------------------------
 *          1/6         2/3       1/6
 *          0           1         0
 * </pre>
 * where the second to last row is the 3rd-order propagated solution and
 * the last row is the 2nd-order midpoint used for the error estimate.
 *
 * The following documentation is pulled from Simbody's implementation
 * of this integrator:
 * "This is a 3-stage, first-same-as-last (FSAL) 3rd order method which
 * gives us an embedded 2nd order method as well, so we can extract
 * a 3rd-order error estimate for the 2nd-order result, which error
 * estimate can then be used for step size control, since it will
 * behave as h^3. We then propagate the 3rd order result (whose error
 * is unknown), which Hairer calls 'local extrapolation'.
 * We call the initial state (t0,y0) and want (t0+h,y1). We are
 * given the initial derivative f0=f(t0,y0), which most likely
 * is left over from an evaluation at the end of the last step."
 *
 * - [Butcher, 1987] J. C. Butcher. The Numerical Analysis of Ordinary
 *   Differential Equations. John Wiley & Sons, 1987. p. 325.
 * - [Hairer, 1993] E. Hairer, S. Noersett, and G. Wanner. Solving ODEs I. 2nd
 *   rev. ed. Springer, 1993. p. 166.
 */
template <class T>
class RungeKutta3Integrator : public IntegratorBase<T> {
 public:
  ~RungeKutta3Integrator() override = default;

  // Disable copy, assign, and move.
  RungeKutta3Integrator(const RungeKutta3Integrator<T>& other) = delete;
  RungeKutta3Integrator& operator=(const RungeKutta3Integrator<T>& other) =
      delete;

  explicit RungeKutta3Integrator(const System<T>& system,
                                 Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    derivs0_ = system.AllocateTimeDerivatives();
    derivs1_ = system.AllocateTimeDerivatives();
    derivs2_ = system.AllocateTimeDerivatives();
  }

  /**
   * The integrator supports error estimation.
   */
  bool supports_error_estimation() const override { return true; }

  /// This integrator provides third order error estimates.
  int get_error_estimate_order() const override { return 3; }

 private:
  void DoInitialize() override;
  std::pair<bool, T> DoStepOnceAtMost(const T& max_dt) override;
  void DoStepOnceFixedSize(const T& dt) override;

  // Vector used in error estimate calculations.
  VectorX<T> err_est_vec_;

  // These are pre-allocated temporaries for use by integration. They store
  // the derivatives computed at various points within the integration
  // interval.
  std::unique_ptr<ContinuousState<T>> derivs0_, derivs1_, derivs2_;
};
}  // namespace systems
}  // namespace drake
