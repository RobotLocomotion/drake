#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

/// This class encapsulates the compliant contact model force computations as
/// described in detail in @ref drake_contacts.
template <typename T>
class CompliantContactModel {
 public :
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompliantContactModel)

  /// Instantiates a %CompliantContactModel from a Multi-Body Dynamics (MBD)
  /// model of the world in `tree`.  `tree` must not be `nullptr`.
  /// \param tree tree the kinematic / dynamic model for which the compliant
  /// contact forces are to be computed.
  CompliantContactModel(const RigidBodyTree<T>& tree);

  /// Computes the generalized forces on all bodies due to contact.
  ///
  /// @param kinsol         The kinematics of the rigid body system at the time
  ///                       of contact evaluation.
  /// @param[out] contacts  The optional contact results.  If non-null, stores
  ///                       the contact information for consuming on the output
  ///                       port.
  /// @returns              The generalized forces across all the bodies due to
  ///                       contact response.
  VectorX<T> ComputeContactForce(const KinematicsCache<T>& kinsol,
                                 ContactResults<T>* contacts = nullptr) const;

  // TODO(SeanCurtis-TRI): Link to documentation explaining these parameters
  // in detail.  To come in a subsequent PR.
  /// Sets only the parameters for *normal* contact.  This is a convenience
  /// function to allow for more targeted parameter tuning.
  void set_normal_contact_parameters(double penetration_stiffness,
                                     double dissipation);

  /// Sets only the parameters for *friction* contact.  This is a convenience
  /// function to allow for more targeted parameter tuning.
  void set_friction_contact_parameters(double static_friction_coef,
                                       double dynamic_friction_coef,
                                       double v_stiction_tolerance);
 private:
  // Computes the friction coefficient based on the relative tangential
  // *speed* of the contact point on Ac relative to B (expressed in B), v_BAc.
  //
  // See contact_model_doxygen.h @section tangent_force for details.
  T ComputeFrictionCoefficient(T v_tangent_BAc) const;

  // Evaluates an S-shaped quintic curve, f(x), mapping the domain [0, 1] to the
  // range [0, 1] where the f''(0) = f''(1) = f'(0) = f'(1) = 0.
  static T step5(T x);

  /// Creates a right-handed local basis from a z-axis. Defines an arbitrary x-
  /// and y-axis such that the basis is orthonormal. The basis is R_WL, where W
  /// is the frame in which the z-axis is expressed and L is a local basis such
  /// that v_W = R_WL * v_L.
  ///
  /// @param[in] z_axis_W   The vector defining the basis's z-axis expressed
  ///                       in frame W.
  /// @retval R_WL          The computed basis.
  static Matrix3<T> ComputeBasisFromZ(const Vector3<T>& z_axis_W);

  // Some parameters defining the contact.
  // TODO(amcastro-tri): Implement contact materials for the RBT engine.
  // These default values are all semi-arbitrary.  They seem to produce,
  // generally, plausible results. They are in *no* way universally valid or
  // meaningful.
  T penetration_stiffness_{10000.0};
  T dissipation_{2};
  // Note: this is the *inverse* of the v_stiction_tolerance parameter to
  // optimize for the division.
  T inv_v_stiction_tolerance_{100};  // inverse of 1 cm/s.
  T static_friction_coef_{0.9};
  T dynamic_friction_coef_{0.5};

  const RigidBodyTree<T>* tree_{nullptr};
};

} // namespace systems
} // namespace drake