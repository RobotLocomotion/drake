#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {

/// Configuration structure for the DoorHinge.
struct DoorHingeConfig {
  /// qs₀ measured outward from the closed position [radian].
  double spring_zero_angle_rad{};
  /// k_ts torsional spring constant measured toward the spring zero angle
  /// [Nm/rad]. It should be non-negative.
  double spring_constant{};
  /// k_df maximum dynamic friction torque measured opposite direction of motion
  /// [Nm]. It should be non-negative.
  double dynamic_friction_torque{};
  /// k_sf maximum static friction measured opposite direction of motion [Nm].
  /// It should be non-negative.
  double static_friction_torque{};
  /// k_vf viscous friction measured opposite direction of motion [Nm]. It
  /// should be non-negative.
  double viscous_friction{};
  /// qc₀ measured from closed (q=0) position [radian]. It should be
  /// non-negative.
  double catch_width{};
  /// k_c maximum catch torque applied over `catch_width` [Nm]. It should be
  /// non-negative.
  double catch_torque{};

  /// k_q̇₀ motion threshold to start to apply friction torques [rad/s]. It
  /// should be non-negative. Realistic frictional force is very stiff,
  /// reversing entirely over zero change in position or velocity, which kills
  /// integrators.  We approximate it with a continuous function.  This constant
  /// [rad/s] is the scaling factor on that function -- very approximately the
  /// rad/s at which half of the full frictional force is applied.  This number
  /// is nonphysical; make it small but not so small that the simulation
  /// vibrates or explodes.
  double motion_threshold{};

  /// Initialize to empirically reasonable values measured approximately by
  /// banging on the door of a dishwasher with a force gauge.
  DoorHingeConfig()
      : spring_zero_angle_rad(-1.5),
        spring_constant(1.5),
        dynamic_friction_torque(3),  // See note above.
        static_friction_torque(2),
        viscous_friction(1),  // In addition to what is in the sdf.
        catch_width(0.02),
        catch_torque(15),
        motion_threshold(0.001) {}
};

/// This %ForceElement models a revolute DoorHinge joint that could exhibit
/// different force/torque characteristics at different states due to the
/// existence of different type of torques on the joint. This class implements
/// a "christmas tree" accumulation of these different torques in an empirical
/// and unprincipled way. Specifically, different curves are assigned to
/// different torques to mimic their evolution based on the joint state and
/// some prespecified parameters.
///
/// Torques considered in this implementation include:
///   * torsional spring torque (τ_ts) -- position dependent
///   * catch torque            (τ_c)  -- position dependent
///   * dynamic friction torque (τ_df) -- velocity dependent
///   * static friction torque  (τ_sf) -- velocity dependent
///   * viscous friction torque (τ_vf) -- velocity dependent
///
/// We then implement two curves to approximate the progression of different
/// torques. A curve `s(t, x) = tanh(x/t)` uses the `tanh` function to
/// approximate a step curve ({`x<0`: -1 ; `x>0`: 1}) outside of `-t < x < t`.
/// The curve `doublet(t, x) = 2 * s * (1 − s²)` is the second derivative of `s`
/// scaled by `-t²`, which yields a lump at negative `x` that integrates to -1
/// and a lump at positive `x` that integrates to 1. Finally, the total
/// external torque on the hinge joint would be:
///
///  `τ = τ_ts + τ_c + τ_df + τ_sf + τ_vf`.
///
/// where `τ_ts = -k_ts * (q − qs₀)`, `τ_c = k_c * doublet(qc₀/2, q − qc₀/2)`,
///  `τ_df = -k_df * s(k_q̇₀, q̇)`, `τ_sf = -k_sf * doublet(k_q̇₀, q̇)` and
/// `τ_vf = -k_vf * q̇`. The door is assumed to be closed at `q=0`, opening
/// in the positive-q direction. Note that, the sign of the torques depends on
/// two elements: one is the sign of the torque related constants and another
/// one is the sign of the assigned curves. For example, as defined above, the
/// static friction torque `τ_sf` should be opposite to the direction of the
/// velocity q̇. The catch torque `τ_c` should be negative when `q < qc₀/2` and
/// positive otherwise. This class applies all hinge-originating forces, so it
/// can be used instead of the SDF viscous damping. The users could change the
/// values of these different elements to obtain different characteristics for
/// the DoorHinge joint that the users want to model. A jupyter notebook tool
/// is also provided to help the users visualize the curves and design
/// parameters.  Run
/// `bazel run //bindings/pydrake/multibody:examples/door_hinge_inspector`
/// to bring up the notebook.
///
/// **To give an example**, a common dishwasher door has a frictional torque
/// sufficient for it to rest motionless at any angle, a catch at the top to
/// hold it in place, a dashpot (viscous friction source) to prevent it from
/// swinging too fast, and a spring to counteract some of its mass. Two
/// figures are provided to illustrate the dishwasher DoorHinge torque with the
/// given default parameters. Figure 1 shows the static characteristic of the
/// dishwasher door. At q = 0, there exists a negative catch torque to prevent
/// the door from moving. After that, the torsional spring torque will dominate
/// to compensate part of the door gravity. Figure 2 shows the dynamic feature
/// of the dishwasher door at q = 30 deg. It shows the door can be closed easily
/// since the torque is small when the velocity is negative. However, whenever
/// the door intends to open further, there will be a counter torque to prevent
/// that movement, which therefore keeps the door at rest. Note that, due to
/// the gravity, the dishwasher door will be fully open eventually. This
/// process can be really slow because of the default `motion_threshold` is
/// set to be very small. You can change the `motion_threshold` parameter to
/// adjust the time.
/// @image html drake/multibody/tree/images/torque_vs_angle.svg "Figure 1"
/// @image html drake/multibody/tree/images/torque_vs_velocity.svg "Figure 2"
///
/// @tparam_default_scalar
template <typename T>
class DoorHinge final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DoorHinge)

  /// Constructs a hinge force element with parameters @p config applied to the
  /// specified @p joint. It will throw an exception if the DoorHingeConfig is
  /// invalid.
  DoorHinge(const RevoluteJoint<T>& joint, const DoorHingeConfig& config);

  const RevoluteJoint<T>& joint() const;

  const DoorHingeConfig& config() const { return config_; }

  T CalcPotentialEnergy(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>&) const override;

  T CalcConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>&,
      const internal::VelocityKinematicsCache<T>&) const override;

  T CalcNonConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>&,
      const internal::VelocityKinematicsCache<T>&) const override;

  /**
   @anchor door_hinge_advanced
   @name Advanced Functions
    Convenience functions for evaluating %DoorHinge operations without worrying
    about context-related issues.
   @{
  */

  /// Calculates the total frictional torque with the given @p angular_rate
  /// and the internal DoorHingeConfig.
  T CalcHingeFrictionalTorque(const T& angular_rate) const;

  /// Calculate the total spring related torque with the given @p angle
  /// and the internal DoorHingeConfig.
  T CalcHingeSpringTorque(const T& angle) const;

  /// Calculate the total torque with the given @p angle and
  /// @p angular_rate and the internal DoorHingeConfig.
  T CalcHingeTorque(const T& angle, const T& angular_rate) const;

  /// Calculate the total conservative power with the given @p angle,
  /// @p angular_rate, and the internal DoorHingeConfig.
  T CalcHingeConservativePower(const T& angle, const T& angular_rate) const;

  /// Calculate the total non-conservative power with the given
  /// @p angular_rate and the internal DoorHingeConfig.
  T CalcHingeNonConservativePower(const T& angular_rate) const;

  /// Calculate the total potential energy of the DoorHinge ForceElement with
  /// the given @p angle and the internal DoorHingeConfig.
  T CalcHingeStoredEnergy(const T& angle) const;
  /// @}

 protected:
  void DoCalcAndAddForceContribution(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>&,
      const internal::VelocityKinematicsCache<T>&,
      MultibodyForces<T>* forces) const override;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>&) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>&) const override;

  std::unique_ptr<ForceElement<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;

 private:
  // For unit test only.
  friend class DoorHingeTest;

  template <typename U>
  friend class DoorHinge;

  DoorHinge(ModelInstanceIndex model_instance, JointIndex joint_index,
            const DoorHingeConfig& config);

  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedClone(
      const internal::MultibodyTree<ToScalar>&) const;

  const JointIndex joint_index_;
  const DoorHingeConfig config_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::DoorHinge)
