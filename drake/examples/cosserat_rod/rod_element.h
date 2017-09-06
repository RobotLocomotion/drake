#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/force_element.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;

/// This ForceElement allows to model the effect of a uniform gravity field.
/// This gravity fields acts on all bodies in the MultibodyTree model.
///
/// References:
///  - [Linn et al. 2013]. Linn, J., Lang, H. and Tuganov, A., 2013.
///    Geometrically exact Cosserat rods with Kelvin–Voigt type viscous damping.
///    Mechanical Sciences, 4(1), pp.79-96.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class RodElement : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RodElement)

  /// Assumptions:
  ///  1. Body frames are located at their center of mass.
  ///  2. Body frame third axis, d3, goes along its axial direction.
  /// @param B1
  ///   Bending stiffness about d1, [N m]. B1 = E * I1.
  /// @param B2
  ///   Bending stiffness about d2, [N m]. B2 = E * I2.
  /// @param C
  ///   Twisting stiffness about d3, [N m]. C = G * Jt, where Jt = I3 in
  ///   the case of circular or annular cross sections only but it is otherwise
  ///   smaller than this value due to the presence of out-of-plane warping of
  ///   the cross sections.
  ///   According to Nikolai's inequality the special case of an elliptic cross
  ///   section maximizes torsional rigidity. For elliptical cross sections we
  ///   have C/G = Jt = πa³b³/(a²+b²) = 4I₁I₂/(I₁+I₂) and therefore Jt/I₃ ≤ 1.
  ///   See references in [Linn et al. 2013].
  /// @param tau_bending
  ///   time retardation constant for bending (extension), in seconds.
  /// @param tau_twisting
  ///   time retardation constant for twisting (shear), in seconds.
  ///
  /// @note Theoretically, for incompressible materials like rubber with Poisson
  /// ratio ν = 0.5, the time retardation constants for internal dissipation are
  /// equal i.e. tau_bending = tau_twisting.
  /// @note Young modulus E and Shear modulus G are related by E = 2G(1+ν) a
  /// relation that can be used to reduce the number of parameters in system
  /// identification experiments.
  explicit RodElement(
      const Body<T>& body1, double length1,
      const Body<T>& body2, double length2,
      double B1, double B2, double C,
      double tau_bending, double tau_twisting);

  T CalcPotentialEnergy(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const final;

  T CalcConservativePower(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const final;

  T CalcNonConservativePower(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const final;

 protected:
  void DoCalcAndAddForceContribution(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      std::vector<SpatialForce<T>>* F_B_W,
      Eigen::Ref<VectorX<T>> tau) const final;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  BodyIndex body_i_index_;
  double length1_;
  BodyIndex body_ip_index_;
  double length2_;
  // Bending stiffness coefficients.
  double B1_, B2_, C_;

  // Internal dissipation is modeled with a Kelvin-Voigt type viscous damping.
  // A really good work on this model for Cosserat rods is presented by
  // [Linn et al. 2013].
  // Linn, J., Lang, H. and Tuganov, A., 2013. Geometrically exact Cosserat
  // rods with Kelvin–Voigt type viscous damping. Mechanical Sciences, 4(1),
  // pp.79-96.
  // NOTE: for nu = 1/2 (incompressible) we have tau_bending = tau_twisting.
  double tau_bending_;  // time retardation constant for bending (extension).
  double tau_twisting_;  // time retardation constant for torsion (shear).
  // Length of the rod at i + 1/2.
  double length_ih_;
};

}  // namespace multibody
}  // namespace drake
