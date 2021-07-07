#pragma once

namespace drake {
namespace multibody {
namespace fem {
/** A simple viscous Rayleigh damping model. The resulting damping matrix is a
nonnegative linear combination of mass and stiffness matrices. Namely, D = αM +
βK where α and β are nonnegative. The damping ratio ξ for a given frequency of
the mode of vibration ω can be calculated by (α/ω + βω)/2. Noticeably, the
contribution by the stiffness term βK is proportional to the frequency of the
mode while the damping ratio contributed by the mass term αM is inversely
proportional to the frequency. Furthermore, one should note that the mass
proportional damping damps rigid body motions and should therefore be kept small
in general.
@tparam_nonsymbolic_scalar T */
template<typename T>
class DampingModel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DampingModel);
  /** Constructs a Rayleigh damping model with the given mass and stiffness
   damping coefficients.
   @throw std::exception if either `mass_coeff` or `stiffness_coeff` is
   negative. */
  DampingModel(T mass_coeff, T stiffness_coeff)
      : mass_coeff_(mass_coeff), stiffness_coeff_(stiffness_coeff) {
    if (mass_coeff < 0.0 || stiffness_coeff < 0.0) {
      throw std::logic_error(
          "Mass and stiffness damping coefficients must be non-negative.");
    }
  }

  const T& mass_coeff() const { return mass_coeff_; }
  const T& stiffness_coeff() const { return stiffness_coeff_; }

 private:
  T mass_coeff_;
  T stiffness_coeff_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
