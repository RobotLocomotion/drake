#pragma once

#include "drake/multibody/fem/discrete_time_integrator.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Implements the interface DiscreteTimeIntegrator with Newmark-beta time
 integration scheme. Given the value for the next time step acceleration `a`,
 the states are calculated from states from the previous time step according to
 the following equations:

      v = vₙ + dt ⋅ (γ ⋅ a + (1−γ) ⋅ aₙ)
      x = xₙ + dt ⋅ vₙ + dt² ⋅ [β ⋅ a + (0.5−β) ⋅ aₙ].

 Note that the scheme is unconditionally unstable for gamma < 0.5 and therefore
 we require gamma >= 0.5.
 See [Newmark, 1959] for the original reference for the method.

 [Newmark, 1959] Newmark, Nathan M. "A method of computation for structural
 dynamics." Journal of the engineering mechanics division 85.3 (1959): 67-94.
 @tparam_nonsymbolic_scalar */
template <typename T>
class NewmarkScheme : public DiscreteTimeIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NewmarkScheme);

  ~NewmarkScheme() = default;

 protected:
  NewmarkScheme() = default;

  /* Constructs a Newmark scheme with the provided timestep, `gamma` and `beta`.
   @pre dt > 0.
   @pre 0.5 <= gamma <= 1.
   @pre 0 <= beta <= 0.5. */
  NewmarkScheme(double dt, double gamma, double beta)
      : dt_(dt), gamma_(gamma), beta_(beta) {
    DRAKE_DEMAND(dt > 0);
    DRAKE_DEMAND(0.5 <= gamma && gamma <= 1);
    DRAKE_DEMAND(0 <= beta && beta <= 0.5);
  }

  double dt() const { return dt_; }
  double gamma() const { return gamma_; }
  double beta() const { return beta_; }

  double dt_{0};
  /* Default values for gamma and beta are set to those of the mid-point rule.
   */
  double gamma_{0.5};
  double beta_{0.25};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
