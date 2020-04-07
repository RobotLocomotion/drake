#pragma once

#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A third-order Runge Kutta integrator with a third order error estimate.
 *
 * For a discussion of this Runge-Kutta method, see [Butcher, 1987]. The
 * embedded error estimate was derived using the method mentioned in
 * [Hairer, 1993].
 *
 * The Butcher tableau for this integrator follows:
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
 *
 * @tparam_nonsymbolic_scalar
 * @ingroup integrators
 */
template <class T>
class RungeKutta3Integrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RungeKutta3Integrator)

  ~RungeKutta3Integrator() override = default;

  explicit RungeKutta3Integrator(const System<T>& system,
                                 Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    derivs0_ = system.AllocateTimeDerivatives();
    derivs1_ = system.AllocateTimeDerivatives();
    err_est_vec_.resize(derivs0_->size());
    save_xc0_.resize(derivs0_->size());
  }

  /**
   * The integrator supports error estimation.
   */
  bool supports_error_estimation() const override { return true; }

  /// This integrator provides third order error estimates.
  int get_error_estimate_order() const override { return 3; }

 private:
  void DoInitialize() override;
  bool DoStep(const T& h) override;

  // Vector used in error estimate calculations.
  VectorX<T> err_est_vec_;

  // Vector used to save initial value of xc.
  VectorX<T> save_xc0_;

  // These are pre-allocated temporaries for use by integration. They store
  // the derivatives computed at various points within the integration
  // interval.
  std::unique_ptr<ContinuousState<T>> derivs0_, derivs1_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::RungeKutta3Integrator)
