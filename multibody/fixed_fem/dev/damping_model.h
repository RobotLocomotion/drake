#pragma once

namespace drake {
namespace multibody {
namespace fixed_fem {
/* A simple viscous Rayleigh damping model. The resulting damping matrix is a
linear combination of mass and stiffness matrices. Namely, D = αM + βK. The
damping ratio ξ for a given frequency of the mode of vibration ω can be
calculated by (α/ω + βω)/2. Noticably, the contribution by the stiffness term βK
is proportional to the frequency of the mode while the damping ratio contributed
by the mass term αM is inversely proportional to the frequency. Furthermore, one
should note that the mass proportional damping damps rigid body motions and
should therefore be kept small in general. */
struct DampingModel {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DampingModel);
  DampingModel(double mass_damping_in, double stiffness_damping_in)
      : mass_damping(mass_damping_in), stiffness_damping(stiffness_damping_in) {
    if (mass_damping_in < 0.0 || stiffness_damping_in < 0.0) {
      throw std::logic_error(
          "Mass and stiffness damping coefficients must be non-negative.");
    }
  }
  double mass_damping;
  double stiffness_damping;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
