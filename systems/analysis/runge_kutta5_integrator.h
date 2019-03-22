#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A fifth-order, seven-stage, first-same-as-last (FSAL) Runge Kutta integrator
 * with a fourth order error estimate.
 * @tparam T A double or autodiff type.
 *
 * This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
 * this class, please refer to https://drake.mit.edu/cxx_inl.html.
 *
 * Instantiated templates for the following kinds of T's are provided:
 *
 * - double
 * - AutoDiffXd
 *
 * For a discussion of this Runge-Kutta method, see [Hairer, 1993]. The
 * embedded error estimate was derived using the method mentioned in
 * [Hairer, 1993].
 *
 * The Butcher tableaux for this integrator follows:
 * <pre>
 * 0    |
 * 1/5  | 1/5
 * 3/10 | 3/40        9/40
 * 4/5  | 44/45       -56/15        32/9
 * 8/9  | 19372/6561 	−25360/2187   64448/6561  −212/729
 * 1    | 9017/3168   −355/33       46732/5247  49/176    −5103/18656
 * 1    | 35/384      0             500/1113    125/192   −2187/6784 	  11/84
 * -----------------------------------------------------------------------------------
 *        35/384      0             500/1113 	 125/192 	 −2187/6784 	  11/84     0
 *        5179/57600 	0             7571/16695 393/640 	 −92097/339200  187/2100 	1/40
 * </pre>
 * where the second to last row is the 5th-order (propagated) solution and
 * the last row gives a 4th-order accurate solution which, when subtracted
 * from the propagated solution, yields the 4th-order error estimate of the
 * error.
 *
 * - [Hairer, 1993] E. Hairer, S. Noersett, and G. Wanner. Solving ODEs I. 2nd
 *   rev. ed. Springer, 1993. p. 166.
 */
template <class T>
class RungeKutta5Integrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RungeKutta5Integrator)

  ~RungeKutta5Integrator() override = default;

  explicit RungeKutta5Integrator(const System<T>& system,
                                 Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    derivs1_ = system.AllocateTimeDerivatives();
    derivs2_ = system.AllocateTimeDerivatives();
    derivs3_ = system.AllocateTimeDerivatives();
    derivs4_ = system.AllocateTimeDerivatives();
    derivs5_ = system.AllocateTimeDerivatives();
    derivs6_ = system.AllocateTimeDerivatives();
    derivs7_ = system.AllocateTimeDerivatives();
    err_est_vec_.resize(derivs1_->size());
    save_xc0_.resize(derivs1_->size());
  }

  /**
   * The integrator supports error estimation.
   */
  bool supports_error_estimation() const override { return true; }

  /// This integrator provides fourth order error estimates.
  int get_error_estimate_order() const override { return 4; }

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
  std::unique_ptr<ContinuousState<T>> derivs1_, derivs2_, derivs3_, derivs4_,
      derivs5_, derivs6_, derivs7_;
};
}  // namespace systems
}  // namespace drake
