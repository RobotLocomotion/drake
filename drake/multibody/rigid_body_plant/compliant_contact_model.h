#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

/// The set of parameters for the compliant contact model. These values affect
/// all contacts in the simulation session. In some sense, they are related
/// to the scale of the simulation. The values must all be _strictly_ positive.
/// See @ref drake_contacts for details.
struct CompliantContactModelParameters {
  double v_stiction_tolerance{1e-2};      // 1 cm/s (in m/s).
  double characteristic_area{2e-4};       // Two square-cm (in m²).
};

/// This class encapsulates the compliant contact model force computations as
/// described in detail in @ref drake_contacts.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
template <typename T>
class CompliantContactModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompliantContactModel)

  /// Instantiates a %CompliantContactModel.
  CompliantContactModel() = default;

  /// Computes the generalized forces on all bodies due to contact.
  ///
  /// @param tree           A Multibody Dynamics (MBD) model of the world.
  /// @param kinsol         The kinematics of the rigid body system at the time
  ///                       of contact evaluation.
  /// @param[out] contacts  The optional contact results.  If non-null, stores
  ///                       the contact information for consuming on the output
  ///                       port.
  /// @returns              The generalized forces across all the bodies due to
  ///                       contact response.
  VectorX<T> ComputeContactForce(
      const RigidBodyTree<T>& tree,
      const KinematicsCache<T>& kinsol,
      ContactResults<T>* contacts = nullptr) const;

  /// Defines the default material property values for this model instance.
  /// All elements with default-configured values will use the values in the
  /// provided property set. This can be invoked before or after parsing
  /// SDF/URDF files; all fields that were left unspecified will default to
  /// these values.
  /// See @ref drake_contact and CompliantMaterial for elaboration on
  /// these values.
  void set_default_material(const CompliantMaterial& material);

  const CompliantMaterial& default_material() const {
    return default_material_;
  }

  /// Configures the model parameters -- these are the global model values that
  /// affect all contacts. If values are outside of valid ranges, the program
  /// aborts. (See CompliantContactParameters for details on valid ranges.)
  void set_model_parameters(const CompliantContactModelParameters& values);

 private:
  // Computes the friction coefficient based on the relative tangential
  // *speed* of the contact point on A relative to B (expressed in B), v_BAc.
  //
  // See contact_model_doxygen.h @section tangent_force for details.
  T ComputeFrictionCoefficient(
      const T& v_tangent_BAc,
      const CompliantMaterial& parameters) const;

  // Evaluates an S-shaped quintic curve, f(x), mapping the domain [0, 1] to the
  // range [0, 1] where the f''(0) = f''(1) = f'(0) = f'(1) = 0.
  static T step5(const T& x);

  // Given two collision elements (with their own defined compliant material
  // properties, computes the _derived_ parameters for the _contact_. Returns
  // The portion of the squish attributable to Element `a` (sₐ). Element `b`'s
  // squish factor is simply 1 - sₐ. See contact_model_doxygen.h for details.
  // @param[in] a            The first element in the contact.
  // @param[in] b            The second element in the contact.
  // @param[out] parameters  The net _contact_ parameters.
  // @retval sₐ  The "squish" factor of Element `a` -- the fraction of the full
  //             penetration deformation that `a` experiences.
  double CalcContactParameters(
      const multibody::collision::Element& a,
      const multibody::collision::Element& b,
      CompliantMaterial* parameters) const;

  // Note: this is the *inverse* of the v_stiction_tolerance parameter to
  // optimize for the division.
  double inv_v_stiction_tolerance_{
      1.0 / CompliantContactModelParameters().v_stiction_tolerance};
  double characteristic_area_{
      CompliantContactModelParameters().characteristic_area};

  // The default compliant material properties for *this* model instance.
  // By default, it uses all hard-coded values.
  CompliantMaterial default_material_;
};

}  // namespace systems
}  // namespace drake
