#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/hydroelastic_quadrature_point_data.h"

namespace drake {
namespace multibody {

namespace internal {

/*
 A class for computing the spatial forces and related reporting data (like
 pressure, traction, and slip) on rigid bodies under the hydroelastic contact
 model, as described in:

 [Elandt, 2019]  R. Elandt, E. Drumwright, M. Sherman, and A. Ruina.
 A pressure field model for fast, robust approximation of net contact force and
 moment between nominally rigid objects. Proc. IEEE/RSJ Intl. Conf. on
 Intelligent Robots and Systems (IROS), 2019.
 */
template <typename T>
class HydroelasticTractionCalculator {
 public:
  // Set of common quantities used through hydroelastic traction calculations.
  // Documentation for parameter names (minus the `_in`
  // suffixes) can be found in the corresponding member documentation.
  struct Data {
    Data(
        const math::RigidTransform<T>& X_WA_in,
        const math::RigidTransform<T>& X_WB_in,
        const SpatialVelocity<T>& V_WA_in,
        const SpatialVelocity<T>& V_WB_in,
        const geometry::ContactSurface<T>* surface_in) :
            X_WA(X_WA_in), X_WB(X_WB_in), V_WA(V_WA_in), V_WB(V_WB_in),
            surface(*surface_in),
            p_WC(surface_in->mesh_W().centroid()) {
      DRAKE_DEMAND(surface_in);
    }

    // The pose of Body A (the body that Geometry `surface.M_id()` in the
    // contact surface is affixed to) in the world frame.
    const math::RigidTransform<T> X_WA;

    // The pose of Body B (the body that Geometry `surface.N_id()` in the
    // contact surface is affixed to) in the world frame.
    const math::RigidTransform<T> X_WB;

    // The spatial velocity of Body A (the body that Geometry
    // `surface.M_id()` in the contact surface is affixed to) at the origin of
    // A's frame, measured and expressed in the world frame.
    const SpatialVelocity<T> V_WA;

    // The spatial velocity of Body B (the body that Geometry
    // `surface.N_id()` in the contact surface is affixed to) at the origin of
    // B's frame, measured and expressed in the world frame.
    const SpatialVelocity<T> V_WB;

    // A reference to the ContactSurface that must be maintained for the life
    // of this object.
    const geometry::ContactSurface<T>& surface;

    // The traction computation needs a point C near the contact surface at
    // which to accumulate forces in a numerically robust way. Our calculations
    // define C to be the centroid of the contact surface, and measure and
    // express this point in the world frame.
    const Vector3<T> p_WC;
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HydroelasticTractionCalculator)

  explicit HydroelasticTractionCalculator(double vslip_regularizer = 1e-6)
      : vslip_regularizer_(vslip_regularizer) {}

  /*
   Gets the regularization parameter used for friction (in m/s). The closer
   that this parameter is to zero, the closer that the regularized friction
   model will approximate Coulomb friction.
   */
  double regularization_scalar() const { return vslip_regularizer_; }

  /*
   Applies the hydroelastic model to two geometries defined in `surface`,
   resulting in a spatial force applied at the centroid of the contact surface.
   This method also provides the data output by the quadrature routine.
   The body frames, A and B, are those to which `surface.M_id()` and
   `surface.N_id()` are affixed, respectively.
   @param data Relevant kinematic data.
   @param dissipation the nonnegative coefficient (in s/m) for dissipating
          energy along the direction of the surface normals.
   @param mu_coulomb the nonnegative coefficient for Coulomb friction.
   @param[out] quadrature_point_data the intermediate data computed by the
               quadrature process. This vector is cleared on entry.
   @param[out] F_Ac_W the spatial force computed by the hydroelastic model that
               acts on the body attached to geometry M (which is affixed to Body
               A) in `data`'s ContactSurface. This spatial force is applied at
               the centroid (Point c) of the contact surface.
   */
  void ComputeSpatialForcesAtCentroidFromHydroelasticModel(
      const Data& data, double dissipation, double mu_coulomb,
      std::vector<HydroelasticQuadraturePointData<T>>*
          quadrature_point_data,
      multibody::SpatialForce<T>* F_Ac_W) const;

  /*
   Shifts the spatial force applied at the centroid of the contact surface
   to equivalent spatial forces applied at the center of the body frames of
   the two interacting bodies. The body frames, A and B, are those to which
   `surface.M_id()` and `surface.N_id()` are affixed, respectively.
   @param data Relevant kinematic data.
   @param F_Ac_W the spatial force computed by the hydroelastic model that acts
          on the body (A) attached to geometry M in `data`'s ContactSurface.
          This spatial force is applied at the centroid (point c) of the contact
          surface.
   @param[output] F_Ao_W the spatial force on Body A, applied at the origin of
                  A's frame, on return.
   @param[output] F_Bo_W the spatial force on Body B, applied at the origin of
                  B's frame, on return.
   */
  void ShiftSpatialForcesAtCentroidToBodyOrigins(
      const Data& data, const SpatialForce<T>& F_Ac_W, SpatialForce<T>* F_Ao_W,
      SpatialForce<T>* F_Bo_W) const;

 private:
  // TODO(edrumwri): Consider methods that expose inner structures of
  // HydroelasticTractionCalculator in HydroelasticReportingTests if we have
  // to "friend" too many testing functions.
  // To allow GTEST to test private functions.
  friend class HydroelasticTractionCalculatorTester;
  friend class MultibodyPlantHydroelasticTractionTests;
  friend class HydroelasticReportingTests;
  friend class HydroelasticReportingTests_LinearTraction_Test;
  friend class HydroelasticReportingTests_LinearSlipVelocity_Test;

  HydroelasticQuadraturePointData<T> CalcTractionAtPoint(
      const Data& data, geometry::SurfaceFaceIndex face_index,
      const typename geometry::SurfaceMesh<T>::Barycentric& Q_barycentric,
      double dissipation, double mu_coulomb) const;

  HydroelasticQuadraturePointData<T> CalcTractionAtQHelper(
      const Data& data, geometry::SurfaceFaceIndex face_index, const T& e,
      const Vector3<T>& nhat_W, double dissipation, double mu_coulomb,
      const Vector3<T>& p_WQ) const;

  multibody::SpatialForce<T> ComputeSpatialTractionAtAcFromTractionAtAq(
      const Data& data, const Vector3<T>& p_WQ,
      const Vector3<T>& traction_Aq_W) const;

  // Computes the function f(x) = atan(x)/x given x² as input. Function
  // atan(x)/x is continuously differentiable everywhere, including
  // x = 0. While this is true analytically, automatic differentiation methods
  // such as AutoDiffXd do not simplify the underlying mathematical expressions
  // as we would by hand. This leads to expressions that are ill-formed at
  // x = 0 and lead to NaN results. Since in our regularized friction model x is
  // the dimensionless norm of a vector, which entails a square root, the naive
  // implementation explicitly writing atan(x)/x leads to a 1/sqrt(x) term that
  // is ill-formed at x=0. This term goes away when the final expression for the
  // derivatives is simplified by hand.
  // This hand crafted method computes an expression of f(x) = atan(x)/x that
  // leads to the exact result of f(x) and its derivative, modulo machine
  // epsilon (in double precision).
  static T CalcAtanXOverXFromXSquared(const T& x_squared) {
    // N.B. We use a compile time iff statement below to provide distinct
    // implementations for numeric and non-numeric scalar types. Numeric scalar
    // types such as double and AutoDiffXd must use the numeric implementation
    // continuous to machine epsilon. Non-numeric types such as
    // symbolic::Expression must use the true expression atan(x)/x.
    if constexpr (scalar_predicate<T>::is_bool) {
      // We are protecting the computation near x = 0 specifically so that
      // numerical values (say double and AutoDiffXd) do not lead to ill-formed
      // expressions with divisions by zero.
      if (x_squared < 1.0e-8) {  // x < 1e-4
        // The taylor expansions for y(x) atan(x)/x and its first derivatives
        // are:
        //   y(x) = 1 - x²/3 +  x⁴/5 + O(x⁶), O(x⁶)  = x⁶/7
        //   y'(x) =   -2x/3 + 4x³/5 + O(x⁵), O'(x⁵) = 6x⁵/7
        // We can compute the relative error in the approxmation as the quotient
        // of the leading terms in the Taylor expansion with the actual value.
        // At x = 1e-4 this amount to the following error estimations:
        //  O(x⁶) / y(x)  = 1.43e-25
        // O'(x⁵) / y'(x) = 1.29e-16
        // which is below (double) machine epsilon for both.
        // Therefore CalcAtanXOverXFromXSquared() is continuous "within machine
        // precision".
        // N.B. We use Horner's method to evaluate the polynomial expression.
        return 1.0 - x_squared * (1.0 / 3.0 - x_squared / 5.0);
      }
    }
    using std::atan;
    using std::sqrt;
    const T x = sqrt(x_squared);
    return atan(x) / x;
  }

  // The parameter (in m/s) for regularizing the Coulomb friction model.
  double vslip_regularizer_{};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

// TODO(edrumwri) instantiate on SymbolicExpression when it no longer
// causes a linker error complaining about an unresolved symbol in SceneGraph.
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::internal::HydroelasticTractionCalculator)
