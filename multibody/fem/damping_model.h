#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace fem {

/** A viscous Rayleigh damping model in which the damping matrix D is a linear
 combination of mass and stiffness matrices, as, D = αM + βK where α and β are
 nonnegative. The damping ratio ζ for a given natural frequency ωₙ of a mode of
 vibration can be calculated as ζ = (α/ωₙ + βωₙ)/2. Notice the contribution of
 the stiffness term βK to the damping matrix D causes ζ to be proportional to a
 mode's natural frequency ωₙ whereas the contribution of the mass term αM to the
 damping matrix D causes ζ to be inversely proportional to a mode's natural
 frequency ωₙ. In the context of rigid body motion (ωₙ = 0), only the αM term
 contributes to the damping matrix D, hence if rigid body motion (or low value
 of ωₙ) is expected, α should be kept small. One way to determine numerical
 values for α and β is to somehow obtain reasonable estimates of the range of
 ωₙ, and then choose numerical values for ζ (e.g 0 ≤ ζ < 0.05). Thereafter
 calculate the associated numerical values of α and β.
 @tparam_nonsymbolic_scalar */
template <typename T>
class DampingModel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DampingModel);

  /** Constructs a Rayleigh damping model by storing the mass coefficient α and
   the stiffness coefficient β that appears in the damping matrix D = αM + βK.
   @throw std::exception if either `mass_coeff_alpha` or `stiffness_coeff_beta`
   is negative. */
  DampingModel(const T& mass_coeff_alpha, const T& stiffness_coeff_beta);

  const T& mass_coeff_alpha() const { return mass_coeff_alpha_; }
  const T& stiffness_coeff_beta() const { return stiffness_coeff_beta_; }

 private:
  T mass_coeff_alpha_{};
  T stiffness_coeff_beta_{};
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::DampingModel);
