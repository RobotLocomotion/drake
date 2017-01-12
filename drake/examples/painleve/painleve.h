#pragma once

#include <utility>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace painleve {

/// Dynamical system representation of the Painleve' Paradox problem, taken
/// from [Stewart 2000]. The Painleve Paradox consists of a rod contacting
/// a planar surface *without impact* and subject to sliding Coulomb friction.
/// The problem is well known to correspond to an *inconsistent rigid contact
/// configuration*, where impulsive forces are necessary to resolve the
/// problem.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Inputs: no inputs.
///
/// States: planar position (state indices 0 and 1) and orientation (state
///         index 2), and planar linear velocity (state indices 3 and 4) and
///         scalar angular velocity (state index 5) in units of m, radians,
///         m/s, and rad/s, respectively. Orientation is measured counter-
///         clockwise with respect to the x-axis.
///
/// Outputs: same as state.
///
/// * [Stewart, 2000]  D. Stewart, "Rigid-Body Dynamics with Friction and
///                    Impact. SIAM Rev., 42(1), 3-39, 2000.
template <typename T>
class Painleve : public systems::LeafSystem<T> {
 public:
  Painleve();

  /// Models impact using an inelastic impact model with friction.
  /// @p new_state is set to the output of the impact model on return.
  void HandleImpact(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* new_state) const;

  /// Gets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  double get_gravitational_acceleration() const { return g_; }

  /// Sets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  void set_gravitational_acceleration(double g) { g_ = g; }

  /// Gets the coefficient of dynamic (sliding) Coulomb friction.
  double get_mu_coulomb() const { return mu_; }

  /// Sets the coefficient of dynamic (sliding) Coulomb friction.
  void set_mu_coulomb(double mu) { mu_ = mu; }

  /// Gets the mass of the rod.
  double get_rod_mass() const { return mass_; }

  /// Sets the mass of the rod.
  void set_rod_mass(double mass) { mass_ = mass; }

  /// Gets the length of the rod.
  double get_rod_length() const { return rod_length_; }

  /// Sets the length of the rod.
  void set_rod_length(double rod_length) { rod_length_ = rod_length; }

  /// Gets the rod moment of inertia.
  double get_rod_moment_of_inertia() const { return J_; }

  /// Sets the rod moment of inertia.
  void set_rod_moment_of_inertia(double J) { J_ = J; }

  /// Checks whether the system is in an impacting state, meaning that the
  /// relative velocity along the contact normal between the rod and the
  /// halfspace is such that the rod will begin interpenetrating the halfspace
  /// at any time Δt in the future (i.e., Δt > 0). If the context does not
  /// correspond to a configuration where the rod and halfspace are contacting,
  /// this method returns `false`.
  bool IsImpacting(const systems::Context<T>& context) const;

 protected:
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

 private:
  Vector2<T> CalcStickingImpactImpulse(const systems::Context<T>& context)
    const;
  Vector2<T> CalcFConeImpactImpulse(const systems::Context<T>& context) const;
  void CalcTimeDerivativesTwoContact(const systems::Context<T>& context,
                                       systems::ContinuousState<T>* derivatives)
                                         const;
  void CalcTimeDerivativesOneContactNoSliding(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const;
  void CalcTimeDerivativesOneContactSliding(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const;
  void SetVelocityDerivatives(const systems::Context<T>& context,
                              systems::VectorBase<T>* const f,
                              T fN, T fF, T xc, T yc) const;
  Vector2<T> CalcStickingContactForces(
      const systems::Context<T>& context) const;
  static std::pair<T, T> CalcRodLowerEndpoint(const T& x,
                                              const T& y,
                                              const int k,
                                              const T& ctheta,
                                              const T& stheta,
                                              const double half_rod_len);

  double mass_{1.0};        // The mass of the rod.
  double rod_length_{1.0};  // The length of the rod.
  double mu_{1000.0};       // The coefficient of friction.
  double g_{-9.81};         // The acceleration due to gravity.
  double J_{1.0};           // The moment of the inertia of the rod.
};

}  // namespace painleve
}  // namespace drake
