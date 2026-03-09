#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {

/// This %ForceElement models a torsional spring attached to a RevoluteJoint
/// and applies a torque to that joint
/// <pre>
///   τ = -k⋅(θ - θ₀)
/// </pre>
/// where θ₀ is the nominal joint position. Note that joint damping exists
/// within the RevoluteJoint itself, and so is not included here.
///
/// The k (stiffness) and θ₀ (nominal angle) specified in the constructor
/// are kept as default values. These parameters are stored within the context
/// and can be accessed and set by context dependent getters/setters.
///
/// @tparam_default_scalar
template <typename T>
class RevoluteSpring final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteSpring);

  /// Constructor for a spring attached to the given joint
  /// @param[in] nominal_angle
  ///   The nominal angle of the spring  θ₀, in radians, at which the spring
  ///   applies no moment.
  /// @param[in] stiffness
  ///   The stiffness k of the spring in N⋅m/rad.
  /// @throws std::exception if `stiffness` is negative.
  RevoluteSpring(const RevoluteJoint<T>& joint, double nominal_angle,
                 double stiffness);

  ~RevoluteSpring() override;

  /// Returns the joint associated with this spring.
  /// @throws std::exception if this element is not associated with a
  ///   MultibodyPlant.
  const RevoluteJoint<T>& joint() const;

  /// Returns the default spring reference angle θ₀ in radians.
  double default_nominal_angle() const { return nominal_angle_; }

  /// Returns the default stiffness constant k in N⋅m/rad.
  double default_stiffness() const { return stiffness_; }

  /// Returns the Context dependent nominal angle θ₀ stored as a
  /// parameter in `context`.
  /// @param[in] context The context storing the state and parameters for the
  /// model to which `this` spring belongs.
  /// @retval returns the nominal angle θ₀ in radians.
  const T& GetNominalAngle(const systems::Context<T>& context) const {
    return context.get_numeric_parameter(spring_parameter_index_).value()[1];
  }

  /// Sets the value of the nominal angle θ₀ for this force
  /// element, stored as a parameter in `context`.
  /// @param[out] context The context storing the state and parameters for the
  /// model to which `this` spring belongs.
  /// @param[in] nominal_angle The nominal angle θ₀ in radians stored within the
  /// context.
  void SetNominalAngle(systems::Context<T>* context,
                       const T& nominal_angle) const {
    context->get_mutable_numeric_parameter(spring_parameter_index_)
        .SetAtIndex(1, nominal_angle);
  }

  /// Returns the Context dependent stiffness coefficient k stored as a
  /// parameter in `context`.
  /// @param[in] context The context storing the state and parameters for the
  /// model to which `this` spring belongs.
  /// @retval returns the stiffness k in N⋅m/rad stored within the context.
  const T& GetStiffness(const systems::Context<T>& context) const {
    return context.get_numeric_parameter(spring_parameter_index_).value()[0];
  }

  /// Sets the value of the linear stiffness coefficient k for this force
  /// element, stored as a parameter in `context`.
  /// @param[out] context The context storing the state and parameters for the
  /// model to which `this` spring belongs.
  /// @param[in] stiffness The stiffness value k with units N⋅m/rad.
  /// @throws std::exception if `stiffness` is negative.
  void SetStiffness(systems::Context<T>* context, const T& stiffness) const {
    DRAKE_THROW_UNLESS(stiffness >= 0);
    context->get_mutable_numeric_parameter(spring_parameter_index_)
        .SetAtIndex(0, stiffness);
  }

  T CalcPotentialEnergy(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc) const override;

  T CalcConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

  T CalcNonConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

 protected:
  void DoCalcAndAddForceContribution(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const override;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<ForceElement<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;

  std::unique_ptr<ForceElement<T>> DoShallowClone() const override;

 private:
  // Implementation for ForceElement::DoDeclareForceElementParameters().
  void DoDeclareForceElementParameters(
      internal::MultibodyTreeSystem<T>* tree_system) final {
    spring_parameter_index_ =
        this->DeclareNumericParameter(tree_system, systems::BasicVector<T>(2));
  }

  // Implementation for ForceElement::DoSetDefaultForceElementParameters().
  void DoSetDefaultForceElementParameters(
      systems::Parameters<T>* parameters) const final {
    // Set the default stiffness and nominal angle parameters.
    systems::BasicVector<T>& spring_parameter =
        parameters->get_mutable_numeric_parameter(spring_parameter_index_);
    spring_parameter.set_value(Vector2<T>(stiffness_, nominal_angle_));
  }

  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U>
  friend class RevoluteSpring;

  RevoluteSpring(ModelInstanceIndex model_instance, JointIndex joint_index,
                 double nominal_angle, double stiffness);

  // Helper method to make a clone templated on ToScalar().
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  const JointIndex joint_index_;
  double nominal_angle_{};
  double stiffness_{};
  systems::NumericParameterIndex spring_parameter_index_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RevoluteSpring);
