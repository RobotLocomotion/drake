#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 A third-order, four-stage, first-same-as-last (FSAL) Runge-Kutta integrator
 with a second order error estimate.

 For a discussion of this Runge-Kutta method, see [Hairer, 1993].
 The Butcher tableau for this integrator follows:
 <pre>
 0    |
 1/2  | 1/2
 3/4  | 0           3/4
 1    | 2/9         1/3      4/9
 -----------------------------------------------------------------------------
        2/9         1/3      4/9     0
        7/24        1/4      1/3     1/8
 </pre>
 where the second to last row is the 3rd-order (propagated) solution and
 the last row gives a 2nd-order accurate solution used for error control.

 - [Bogacki, 1989] P. Bogacki and L. Shampine. "A 3(2) pair of Runge–Kutta
   formulas", Appl. Math. Letters, 2 (4): 321–325, 1989.

 @tparam_nonsymbolic_scalar
 @ingroup integrators
 */
template <class T>
class BogackiShampine3Integrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BogackiShampine3Integrator)

  ~BogackiShampine3Integrator() override = default;

  explicit BogackiShampine3Integrator(const System<T>& system,
      Context<T>* context = nullptr) : IntegratorBase<T>(system, context) {
    derivs1_ = system.AllocateTimeDerivatives();
    derivs2_ = system.AllocateTimeDerivatives();
    derivs3_ = system.AllocateTimeDerivatives();
    err_est_vec_ = std::make_unique<BasicVector<T>>(derivs1_->size());
    save_xc0_.resize(derivs1_->size());
  }

  /**
   * The integrator supports error estimation.
   */
  bool supports_error_estimation() const override { return true; }

  /// The order of the asymptotic term in the error estimate.
  int get_error_estimate_order() const override { return 3; }

 private:
  void DoInitialize() override;
  bool DoStep(const T& h) override;

  // Vector used in error estimate calculations.
  std::unique_ptr<BasicVector<T>> err_est_vec_;

  // Vector used to save initial value of xc.
  VectorX<T> save_xc0_;

  // These are pre-allocated temporaries for use by integration. They store
  // the derivatives computed at various points within the integration
  // interval.
  std::unique_ptr<ContinuousState<T>> derivs1_, derivs2_, derivs3_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::BogackiShampine3Integrator)
