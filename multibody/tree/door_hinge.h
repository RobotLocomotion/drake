#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {

// Drake-less math exposed for easier testing.
namespace internal {
template <typename T>
T hinge_frictional_torque(T angle, T angular_velocity,
                          const DoorHingeConfig& config);

template <typename T>
T hinge_spring_torque(T angle, T angular_velocity,
                      const DoorHingeConfig& config);

template <typename T>
T hinge_torque(T angle, T angular_velocity, const DoorHingeConfig& config);

template <typename T>
T hinge_conservative_power(T angle, T angular_velocity,
                           const DoorHingeConfig& config);

template <typename T>
T hinge_nonconservative_power(T angle, T angular_velocity,
                              const DoorHingeConfig& config);

template <typename T>
T hinge_stored_energy(T angle, T angular_velocity,
                      const DoorHingeConfig& config);
}  // namespace internal

/// Configuration structure for the door hinge.
struct DoorHingeConfig {
  double spring_zero_angle_rad;    //< radians (outward from closed)
  double spring_constant;          //< Nm/rad (toward `spring_zero_angle_rad`)
  double dynamic_friction_torque;  //< Nm (opposite direction of motion)
  double static_friction_torque;   //< Nm (opposite direction of motion)
  double viscous_friction;         //< Nms/rad (Nm per rad/s) (opp. motion)
  double catch_width;              //< radians (from closed (θ=0) position)
  double catch_torque;             //< Nm (applied over `catch_width`)

  /// Realistic frictional force is very stiff, reversing entirely over zero
  /// change in position or velocity, which kills integrators.  We approximate
  /// it with a continuous function.  This constant is the scaling factor on
  /// that function -- very approximately the rad/s at which half of the full
  /// frictional force is applied.  This number is nonphysical; make it small
  /// but not so small that the simulation vibrates or explodes.
  double motion_threshold;  //< rad/s

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

/// This %ForceElement models a revolute door hinge joint that could exhibits
/// different force/torque characterisitcs at different states due to the
/// existence of different type of torques on the joint. This class implements
/// a "christmas tree" accumulation of these different torques in an empirical
/// and unprincipled way. Specifically, different curves are assigned to
/// different torques to mimic their evolution based on the joint state and
/// some prespecified parameters.
///
/// Torques considered in this implementation include:
///   * torsional spring torque -- position dependent
///   * dynamic friction torque -- velocity dependent
///   * static friction torque  -- velocity dependent
///   * viscous friction torque -- velocity dependent
///   * catch torque -- position dependent
///
///  total_external_torque = spring_constant * (q `-` spring_zero_angle_rad)
///               `-` dynamic_friction_torque * sigmoid(q̇, motion_threshold)
///               `-` static_friction_torque * doublet(q̇, motion_threhold)
///               `-` viscous_friction * q̇
///               `-` catch_torque * doublet(q `-` catch_width, catch_width)
///
/// The door is assumed to be closed at q=0, opening in the positive-q
/// direction. This class applies all hinge-originating forces, so it can be
/// used instead of the SDF viscous damping. The users could change the values
/// of these different elements to obtain different characterisitcs for the
/// door hinge joint that the users want to model.
///
/// For example, a common dishwasher door has a frictional torque
/// sufficient for it to rest motionless at any angle, a catch at the top to
/// hold it in place, a dashpot (viscous friction source) to prevent it from
/// swinging too fast, and a spring to counteract some of its mass. The
/// following two figures illustrate the dishwasher door hinge torque with the
/// given default parameters. Figure 1 shows the static characteristic of the
/// dishwasher door. At q = 0, there exists a negative catch torque to prevent
/// the door from moving. After that, the torsional spring torque will dominate
/// to compensate part of the door gravity. Figure 2 shows the dynamic feature
/// of the dishwasher door at q = 30 deg. Whenever the door tries to move,
/// there will be a counter torque to prevent that movement, which therefore
/// keeps the door at rest. Note that, due to the gravity, the dishwasher door
/// will be fully open eventually. However, this process can be really slow
/// because of the default `motion_threshold` is set to be very small.
/// You can change the `motion_threshold` parameter to adjust the time.
/// @image html multibody/tree/images/torque_vs_angle.svg "Figure 1: Illustration of total external torque on the door angle."
/// @image html multibody/tree/images/torque_vs_velocity.svg "Figure 2: Illustration of total external torque on the door angular velocity."
/// A jupytor notebook tool is also provided to help the users visualize the
/// curves and design parameters.
/// TODO(huihua) Add a link to the jupytor notebook.

template <typename T>
class DoorHinge : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DoorHinge)

  /// Construct a hinge force element with parameters @p config applied to the
  /// specified @p joint.
  /// The @p joint is aliased and must remain valid for the lifetime of this
  /// object and its clones.
  ///
  /// Some minimal sanity checking is asserted on the supplied config.
  /// @throws std::exception if `config.spring_constant` is negative.
  /// @throws std::exception if `config.dynamic_friction_torque` is negative.
  /// @throws std::exception if `config.static_friction_torque` is negative.
  /// @throws std::exception if `config.viscous_friction` is negative.
  /// @throws std::exception if `config.catch_width` is negative or zero.
  /// @throws std::exception if `config.motion_threshold` is negative or zero.
  DoorHinge(const RevoluteJoint<T>& joint, const DoorHingeConfig& config)
      : ForceElement<T>(joint.model_instance()),
        joint_(joint),
        config_(config) {
    DRAKE_DEMAND(config_.spring_constant >= 0);
    DRAKE_DEMAND(config_.dynamic_friction_torque >= 0);
    DRAKE_DEMAND(config_.static_friction_torque >= 0);
    DRAKE_DEMAND(config_.viscous_friction >= 0);
    DRAKE_DEMAND(config_.catch_width > 0);
    DRAKE_DEMAND(config_.motion_threshold > 0);
  }

  const RevoluteJoint<T>& joint() const { return joint_; }

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
      // NOLINTNEXTLINE(whitespace/line_length)
      const internal::MultibodyTree<symbolic::Expression>&) const override;

 private:
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedClone(
      const internal::MultibodyTree<ToScalar>&) const;

  const RevoluteJoint<T>& joint_;
  DoorHingeConfig config_;
};

}  // namespace multibody
}  // namespace drake
