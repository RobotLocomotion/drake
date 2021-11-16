#include "drake/multibody/plant/compliant_contact_manager.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"
#include "drake/systems/framework/context.h"

using drake::geometry::GeometryId;
using drake::geometry::PenetrationAsPointPair;
using drake::math::RotationMatrix;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
CompliantContactManager<T>::CompliantContactManager(
    std::unique_ptr<contact_solvers::internal::ContactSolver<T>> contact_solver)
    : contact_solver_(std::move(contact_solver)) {
  DRAKE_DEMAND(contact_solver_ != nullptr);
}

template <typename T>
void CompliantContactManager<T>::DeclareCacheEntries() {
  // N.B. We use xd_ticket() instead of q_ticket() since discrete
  // multibody plant does not have q's, but rather discrete state.
  // Therefore if we make it dependent on q_ticket() the Jacobian only
  // gets evaluated once at the start of the simulation.

  // Cache discrete contact pairs.
  const auto& discrete_contact_pairs_cache_entry = this->DeclareCacheEntry(
      "Discrete contact pairs.",
      systems::ValueProducer(
          this, &CompliantContactManager<T>::CalcDiscreteContactPairs),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.discrete_contact_pairs =
      discrete_contact_pairs_cache_entry.cache_index();

  // Contact Jacobian for the discrete pairs computed with
  // CompliantContactManager::CalcDiscreteContactPairs().
  auto& contact_jacobian_cache_entry = this->DeclareCacheEntry(
      std::string("Contact Jacobian."),
      systems::ValueProducer(
          this, &CompliantContactManager<T>::CalcContactJacobianCache),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.contact_jacobian = contact_jacobian_cache_entry.cache_index();
}

template <typename T>
void CompliantContactManager<T>::CalcContactJacobianCache(
    const systems::Context<T>& context,
    internal::ContactJacobianCache<T>* cache) const {
  DRAKE_DEMAND(cache != nullptr);

  const std::vector<internal::DiscreteContactPair<T>>& contact_pairs =
      EvalDiscreteContactPairs(context);
  const int num_contacts = contact_pairs.size();

  MatrixX<T>& Jc = cache->Jc;
  Jc.resize(3 * num_contacts, plant().num_velocities());

  std::vector<drake::math::RotationMatrix<T>>& R_WC_set = cache->R_WC_list;
  R_WC_set.clear();
  R_WC_set.reserve(num_contacts);

  // Quick no-op exit.
  if (num_contacts == 0) return;

  const int nv = plant().num_velocities();

  // Scratch workspace variables.
  Matrix3X<T> Jv_WAc_W(3, nv);
  Matrix3X<T> Jv_WBc_W(3, nv);
  Matrix3X<T> Jv_AcBc_W(3, nv);

  const Frame<T>& frame_W = plant().world_frame();
  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& point_pair = contact_pairs[icontact];

    const GeometryId geometryA_id = point_pair.id_A;
    const GeometryId geometryB_id = point_pair.id_B;

    BodyIndex bodyA_index = this->geometry_id_to_body_index().at(geometryA_id);
    const Body<T>& bodyA = plant().get_body(bodyA_index);
    BodyIndex bodyB_index = this->geometry_id_to_body_index().at(geometryB_id);
    const Body<T>& bodyB = plant().get_body(bodyB_index);

    // Contact normal from point A into B.
    const Vector3<T>& nhat_W = -point_pair.nhat_BA_W;
    const Vector3<T>& p_WC = point_pair.p_WC;

    // Since v_AcBc_W = v_WBc - v_WAc the relative velocity Jacobian will be:
    //   J_AcBc_W = Jv_WBc_W - Jv_WAc_W.
    // That is the relative velocity at C is v_AcBc_W = J_AcBc_W * v.
    this->internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, bodyA.body_frame(), frame_W, p_WC,
        frame_W, frame_W, &Jv_WAc_W);
    this->internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, bodyB.body_frame(), frame_W, p_WC,
        frame_W, frame_W, &Jv_WBc_W);
    Jv_AcBc_W = Jv_WBc_W - Jv_WAc_W;

    // Define a contact frame C at the contact point such that the z-axis Cz
    // equals nhat_W. The tangent vectors are arbitrary, with the only
    // requirement being that they form a valid right handed basis with nhat_W.
    const math::RotationMatrix<T> R_WC =
        math::RotationMatrix<T>::MakeFromOneVector(nhat_W, 2);
    R_WC_set.push_back(R_WC);

    Jc.template middleRows<3>(3 * icontact).noalias() =
        R_WC.matrix().transpose() * Jv_AcBc_W;
  }
}

template <typename T>
T CompliantContactManager<T>::GetPointContactStiffness(
    geometry::GeometryId id,
    const geometry::SceneGraphInspector<T>& inspector) const {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  // N.B. Here we rely on the resolution of #13289 and #5454 to get properties
  // with the proper scalar type T. This will not work on scalar converted
  // models until those issues are resolved.
  return prop->template GetPropertyOrDefault<T>(
      geometry::internal::kMaterialGroup, geometry::internal::kPointStiffness,
      this->default_contact_stiffness());
}

template <typename T>
T CompliantContactManager<T>::GetDissipationTimeConstant(
    geometry::GeometryId id,
    const geometry::SceneGraphInspector<T>& inspector) const {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  // N.B. Here we rely on the resolution of #13289 and #5454 to get properties
  // with the proper scalar type T. This will not work on scalar converted
  // models until those issues are resolved.
  return prop->template GetPropertyOrDefault<T>(
      geometry::internal::kMaterialGroup, "dissipation_time_constant",
      plant().time_step());
}

template <typename T>
T CompliantContactManager<T>::CombineStiffnesses(const T& k1, const T& k2) {
  // Simple utility to detect 0 / 0. As it is used in this method, denom
  // can only be zero if num is also zero, so we'll simply return zero.
  auto safe_divide = [](const T& num, const T& denom) {
    return denom == 0.0 ? 0.0 : num / denom;
  };
  return safe_divide(k1 * k2, k1 + k2);
}

template <typename T>
T CompliantContactManager<T>::CombineDissipationTimeConstant(const T& tau1,
                                                             const T& tau2) {
  return tau1 + tau2;
}

template <typename T>
void CompliantContactManager<T>::CalcDiscreteContactPairs(
    const systems::Context<T>& context,
    std::vector<internal::DiscreteContactPair<T>>* contact_pairs) const {
  plant().ValidateContext(context);
  DRAKE_DEMAND(contact_pairs != nullptr);

  contact_pairs->clear();
  if (plant().num_collision_geometries() == 0) return;

  const auto contact_model = plant().get_contact_model();

  // We first compute the number of contact pairs so that we can allocate all
  // memory at once.
  // N.B. num_point_pairs = 0 when:
  //   1. There are legitimately no point pairs or,
  //   2. the point pair model is not even in use.
  // We guard for case (2) since EvalPointPairPenetrations() cannot be called
  // when point contact is not used and would otherwise throw an exception.
  int num_point_pairs = 0;  // The number of point contact pairs.
  if (contact_model == ContactModel::kPoint ||
      contact_model == ContactModel::kHydroelasticWithFallback) {
    num_point_pairs = plant().EvalPointPairPenetrations(context).size();
  }

  int num_quadrature_pairs = 0;
  // N.B. For discrete hydro we use a first order quadrature rule.
  // Higher order quadratures are possible, however using a lower order
  // quadrature leads to a smaller number of discrete pairs and therefore a
  // smaller number of constraints in the discrete contact problem.
  const GaussianTriangleQuadratureRule quadrature(1 /* order */);
  const std::vector<double>& wq = quadrature.weights();
  const int num_quad_points = wq.size();
  if (contact_model == ContactModel::kHydroelastic ||
      contact_model == ContactModel::kHydroelasticWithFallback) {
    const std::vector<geometry::ContactSurface<T>>& surfaces =
        this->EvalContactSurfaces(context);
    for (const auto& s : surfaces) {
      const geometry::TriangleSurfaceMesh<T>& mesh = s.mesh_W();
      num_quadrature_pairs += num_quad_points * mesh.num_triangles();
    }
  }
  const int num_contact_pairs = num_point_pairs + num_quadrature_pairs;
  contact_pairs->reserve(num_contact_pairs);
  if (contact_model == ContactModel::kPoint ||
      contact_model == ContactModel::kHydroelasticWithFallback) {
    AppendDiscreteContactPairsForPointContact(context, contact_pairs);
  }
  if (contact_model == ContactModel::kHydroelastic ||
      contact_model == ContactModel::kHydroelasticWithFallback) {
    AppendDiscreteContactPairsForHydroelasticContact(context, contact_pairs);
  }
}

template <typename T>
void CompliantContactManager<T>::AppendDiscreteContactPairsForPointContact(
    const systems::Context<T>& context,
    std::vector<internal::DiscreteContactPair<T>>* result) const {
  std::vector<internal::DiscreteContactPair<T>>& contact_pairs = *result;

  const geometry::QueryObject<T>& query_object =
      this->plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();

  // Fill in the point contact pairs.
  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      plant().EvalPointPairPenetrations(context);
  for (const PenetrationAsPointPair<T>& pair : point_pairs) {
    const T kA = GetPointContactStiffness(pair.id_A, inspector);
    const T kB = GetPointContactStiffness(pair.id_B, inspector);
    const T k = CombineStiffnesses(kA, kB);
    const T tauA = GetDissipationTimeConstant(pair.id_A, inspector);
    const T tauB = GetDissipationTimeConstant(pair.id_B, inspector);
    const T tau = CombineDissipationTimeConstant(tauA, tauB);
    const T d = tau * k;

    // We compute the position of the point contact based on Hertz's theory
    // for contact between two elastic bodies.
    const T denom = kA + kB;
    const T wA = (denom == 0 ? 0.5 : kA / denom);
    const T wB = (denom == 0 ? 0.5 : kB / denom);
    const Vector3<T> p_WC = wA * pair.p_WCa + wB * pair.p_WCb;

    const T phi0 = -pair.depth;
    const T fn0 = -k * phi0;
    contact_pairs.push_back(
        {pair.id_A, pair.id_B, p_WC, pair.nhat_BA_W, phi0, fn0, k, d});
  }
}

template <typename T>
void CompliantContactManager<T>::
    AppendDiscreteContactPairsForHydroelasticContact(
        const systems::Context<T>& context,
        std::vector<internal::DiscreteContactPair<T>>* result) const {
  std::vector<internal::DiscreteContactPair<T>>& contact_pairs = *result;

  // N.B. For discrete hydro we use a first order quadrature rule.
  // Higher order quadratures are possible, however using a lower order
  // quadrature leads to a smaller number of discrete pairs and therefore a
  // smaller number of constraints in the discrete contact problem.
  const GaussianTriangleQuadratureRule quadrature(1 /* order */);
  const std::vector<Vector2<double>>& xi = quadrature.quadrature_points();
  const std::vector<double>& wq = quadrature.weights();
  const int num_quad_points = wq.size();

  const geometry::QueryObject<T>& query_object =
      this->plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();
  const std::vector<geometry::ContactSurface<T>>& surfaces =
      this->EvalContactSurfaces(context);
  for (const auto& s : surfaces) {
    const geometry::TriangleSurfaceMesh<T>& mesh_W = s.mesh_W();
    const T tau_M = GetDissipationTimeConstant(s.id_M(), inspector);
    const T tau_N = GetDissipationTimeConstant(s.id_N(), inspector);
    const T tau = CombineDissipationTimeConstant(tau_M, tau_N);

    for (int face = 0; face < mesh_W.num_triangles(); ++face) {
      const T& Ae = mesh_W.area(face);  // Face element area.

      // We found out that the hydroelastic query might report
      // infinitesimally small triangles (consider for instance an initial
      // condition that perfectly places an object at zero distance from the
      // ground.) While the area of zero sized triangles is not a problem by
      // itself, the badly computed normal on these triangles leads to
      // problems when computing the contact Jacobians (since we need to
      // obtain an orthonormal basis based on that normal.)
      // We therefore ignore infinitesimally small triangles. The tolerance
      // below is somehow arbitrary and could possibly be tightened.
      if (Ae > 1.0e-14) {
        // N.B Assuming rigid-soft contact, and thus only a single pressure
        // gradient is considered to be valid. We first verify this indeed
        // is the case by checking that only one side has gradient
        // information (the volumetric side).
        const bool M_is_soft = s.HasGradE_M();
        const bool N_is_soft = s.HasGradE_N();
        DRAKE_DEMAND(M_is_soft ^ N_is_soft);

        // Pressure gradient always points into the soft geometry by
        // construction.
        const Vector3<T>& grad_pres_W =
            M_is_soft ? s.EvaluateGradE_M_W(face) : s.EvaluateGradE_N_W(face);

        // From ContactSurface's documentation: The normal of each face is
        // guaranteed to point "out of" N and "into" M.
        const Vector3<T>& nhat_W = mesh_W.face_normal(face);
        for (int qp = 0; qp < num_quad_points; ++qp) {
          const Vector3<T> barycentric(xi[qp](0), xi[qp](1),
                                       1.0 - xi[qp](0) - xi[qp](1));
          // Pressure at the quadrature point.
          const T p0 = s.e_MN().Evaluate(face, barycentric);

          // Force contribution by this quadrature point.
          const T fn0 = wq[qp] * Ae * p0;

          // Since the normal always points into M, regardless of which body
          // is soft, we must take into account the change of sign when body
          // N is soft and M is rigid.
          const T sign = M_is_soft ? 1.0 : -1.0;

          // In order to provide some intuition, and though not entirely
          // complete, here we document the first order idea that leads to
          // the discrete hydroelastic approximation used below. In
          // hydroelastics, each quadrature point contributes to the total
          // "elastic" force along the normal direction as:
          //   f₀ₚ = ωₚ Aₑ pₚ
          // where subindex p denotes a quantity evaluated at quadrature
          // point P and subindex e identifies the e-th contact surface
          // element in which the quadrature is being evaluated.
          // Notice f₀ only includes the "elastic" contribution. Dissipation is
          // dealt with by the contact solver. In point contact, stiffness is
          // related to changes in the normal force with changes in the
          // penetration distance. In that spirit, the approximation used here
          // is to define the discrete hydroelastics stiffness as the
          // directional derivative of the scalar force f₀ₚ along the normal
          // direction n̂:
          //   k := ∂f₀ₚ/∂n̂ ≈ ωₚ⋅Aₑ⋅∇pₚ⋅n̂ₚ
          // that is, the variation of the normal force experiences if the
          // quadrature point is pushed inwards in the direction of the
          // normal. Notice that this expression approximates the element
          // area and normal as constant. Keeping normals and Jacobians
          // is a very common approximation in first order methods. Keeping
          // the area constant here is a higher order approximation for
          // inner triangles given that shrinkage of a triangle is related
          // the growth of a neighboring triangle (i.e. the contact surface
          // does not stretch nor shrink). For triangles close to the
          // boundary of the contact surface, this is only a first order
          // approximation.
          //
          // Refer to [Masterjohn, 2021] for details.
          //
          // [Masterjohn, 2021] Masterjohn J., Guoy D., Shepherd J. and Castro
          // A., 2021. Discrete Approximation of Pressure Field Contact Patches.
          // Available at https://arxiv.org/abs/2110.04157.
          const T k = sign * wq[qp] * Ae * grad_pres_W.dot(nhat_W);

          // N.B. The normal is guaranteed to point into M. However, when M
          // is soft, the gradient is not guaranteed to be in the direction
          // of the normal. The geometry code that determines which
          // triangles to keep in the contact surface may keep triangles for
          // which the pressure gradient times normal is negative (see
          // IsFaceNormalInNormalDirection() in contact_surface_utility.cc).
          // Therefore there are cases for which the definition above of k
          // might lead to negative values. We observed that this condition
          // happens sparsely at some of the boundary triangles of the
          // contact surface, while the positive values in inner triangles
          // dominates the overall compliance. In practice we did not
          // observe this to cause stability issues. Since a negative value
          // of k is correct, we decided to keep these contributions.

          // Position of quadrature point Q in the world frame (since mesh_W
          // is measured and expressed in W).
          const Vector3<T> p_WQ =
              mesh_W.CalcCartesianFromBarycentric(face, barycentric);

          // phi < 0 when in penetration.
          const T phi0 = -sign * p0 / grad_pres_W.dot(nhat_W);

          if (k > 0) {
            const T dissipation = tau * k;
            contact_pairs.push_back(
                {s.id_M(), s.id_N(), p_WQ, nhat_W, phi0, fn0, k, dissipation});
          }
        }
      }
    }
  }
}

template <typename T>
const std::vector<internal::DiscreteContactPair<T>>&
CompliantContactManager<T>::EvalDiscreteContactPairs(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.discrete_contact_pairs)
      .template Eval<std::vector<internal::DiscreteContactPair<T>>>(context);
}

template <typename T>
const internal::ContactJacobianCache<T>&
CompliantContactManager<T>::EvalContactJacobianCache(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.contact_jacobian)
      .template Eval<internal::ContactJacobianCache<T>>(context);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::CompliantContactManager);
