#include "drake/multibody/plant/deformable_driver.h"

#include <array>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/contact_solvers/contact_configuration.h"
#include "drake/multibody/fem/dirichlet_boundary_condition.h"
#include "drake/multibody/fem/fem_model.h"
#include "drake/multibody/plant/contact_properties.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/multibody/plant/hydroelastic_quadrature_point_data.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

using drake::geometry::GeometryId;
using drake::geometry::internal::ContactParticipation;
using drake::geometry::internal::DeformableContact;
using drake::geometry::internal::DeformableContactSurface;
using drake::multibody::contact_solvers::internal::Block3x3SparseMatrix;
using drake::multibody::contact_solvers::internal::ContactConfiguration;
using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::contact_solvers::internal::FixedConstraintKinematics;
using drake::multibody::contact_solvers::internal::MatrixBlock;
using drake::multibody::contact_solvers::internal::PartialPermutation;
using drake::multibody::contact_solvers::internal::SapConstraintJacobian;
using drake::multibody::contact_solvers::internal::SchurComplement;
using drake::multibody::contact_solvers::internal::VertexPartialPermutation;
using drake::multibody::fem::FemModel;
using drake::multibody::fem::FemState;
using drake::multibody::fem::internal::DirichletBoundaryCondition;
using drake::multibody::fem::internal::FemSolver;
using drake::systems::Context;
using drake::systems::DependencyTicket;

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
DeformableDriver<T>::DeformableDriver(
    const DeformableModel<T>* deformable_model,
    const DiscreteUpdateManager<T>* manager)
    : deformable_model_(deformable_model), manager_(manager) {
  DRAKE_DEMAND(deformable_model != nullptr);
  DRAKE_DEMAND(manager != nullptr);
}

template <typename T>
DeformableDriver<T>::~DeformableDriver() = default;

template <typename T>
void DeformableDriver<T>::DeclareCacheEntries(
    DiscreteUpdateManager<T>* manager) {
  DRAKE_DEMAND(manager_ == manager);

  /* Our CalcDeformableContact() uses `query_object` (not `context`), and only
   returns deformable contact (not all geometry contact), so we know that this
   ticket is sufficient to cover our EvalDeformableContact(). */
  const systems::DependencyTicket deformable_contact_ticket =
      manager->plant().get_geometry_query_input_port().ticket();

  /* The collection of constraint participation tickets for *all* deformable
    bodies to be filled out in the loop below. */
  std::set<systems::DependencyTicket> constraint_participation_tickets;
  constraint_participation_tickets.emplace(deformable_contact_ticket);

  for (DeformableBodyIndex i(0); i < deformable_model_->num_bodies(); ++i) {
    const DeformableBodyId id = deformable_model_->GetBodyId(i);
    const fem::FemModel<T>& fem_model = deformable_model_->GetFemModel(id);
    std::unique_ptr<fem::FemState<T>> model_state = fem_model.MakeFemState();
    /* Cache entry for current FEM state. */
    const auto& fem_state_cache_entry_ticket =
        manager_->plant()
            .get_cache_entry(
                deformable_model_->GetBody(i).fem_state_cache_index())
            .ticket();

    /* Constraint participation information for each body. */
    ContactParticipation empty_contact_participation(fem_model.num_nodes());
    const auto& constraint_participation_cache_entry =
        manager->DeclareCacheEntry(
            fmt::format("constraint participation of body {}", i),
            systems::ValueProducer(
                empty_contact_participation,
                std::function<void(const Context<T>&, ContactParticipation*)>{
                    [this, i](const Context<T>& context,
                              ContactParticipation* result) {
                      this->CalcConstraintParticipation(context, i, result);
                    }}),
            {deformable_contact_ticket});
    cache_indexes_.constraint_participations.emplace_back(
        constraint_participation_cache_entry.cache_index());
    constraint_participation_tickets.emplace(
        constraint_participation_cache_entry.ticket());

    /* Permutation for participating vertices for each body. */
    const GeometryId g_id =
        deformable_model_->GetGeometryId(deformable_model_->GetBodyId(i));
    const auto& vertex_permutation_cache_entry = manager->DeclareCacheEntry(
        fmt::format("partial permutation for vertices of body {} based on "
                    "participation in contact",
                    i),
        systems::ValueProducer(
            std::function<void(const Context<T>&, VertexPartialPermutation*)>{
                [this, g_id](const Context<T>& context,
                             VertexPartialPermutation* result) {
                  this->CalcPermutation(context, g_id, result);
                }}),
        {constraint_participation_cache_entry.ticket()});
    cache_indexes_.vertex_permutations.emplace(
        g_id, vertex_permutation_cache_entry.cache_index());

    FemSolver<T> model_fem_solver(&fem_model, &deformable_model_->integrator());
    /* Cache entry for free motion FEM state and data. */
    const auto& fem_solver_cache_entry = manager->DeclareCacheEntry(
        fmt::format("FEM solver and data for body with index {}", i),
        systems::ValueProducer(
            model_fem_solver,
            std::function<void(const systems::Context<T>&, FemSolver<T>*)>{
                [this, i](const systems::Context<T>& context,
                          FemSolver<T>* fem_solver) {
                  this->CalcFreeMotionFemSolver(context, i, fem_solver);
                }}),
        /* Free motion velocities can depend on user defined external forces
         which in turn depends on input ports. */
        {fem_state_cache_entry_ticket, vertex_permutation_cache_entry.ticket(),
         systems::System<T>::all_input_ports_ticket(),
         systems::System<T>::all_parameters_ticket()});
    cache_indexes_.fem_solvers.emplace_back(
        fem_solver_cache_entry.cache_index());

    /* Cache entry for FEM state at next time step. */
    const auto& next_fem_state_cache_entry = manager->DeclareCacheEntry(
        fmt::format("FEM state for body with index {} at next time step", i),
        systems::ValueProducer(
            *model_state,
            std::function<void(const systems::Context<T>&, fem::FemState<T>*)>{
                [this, i](const systems::Context<T>& context,
                          fem::FemState<T>* next_fem_state) {
                  this->CalcNextFemState(context, i, next_fem_state);
                }}),
        {systems::System<T>::xd_ticket(),
         systems::System<T>::all_parameters_ticket(),
         systems::System<T>::all_input_ports_ticket(),
         systems::System<T>::time_ticket(),
         systems::System<T>::accuracy_ticket()});
    cache_indexes_.next_fem_states.emplace_back(
        next_fem_state_cache_entry.cache_index());
  }

  const auto& participating_velocity_mux_cache_entry =
      manager->DeclareCacheEntry(
          "multiplexer for participating velocities",
          systems::ValueProducer(
              this, &DeformableDriver<T>::CalcParticipatingVelocityMultiplexer),
          constraint_participation_tickets);
  cache_indexes_.participating_velocity_mux =
      participating_velocity_mux_cache_entry.cache_index();

  auto constraint_participation_and_xd_tickets =
      constraint_participation_tickets;
  constraint_participation_and_xd_tickets.insert(
      systems::System<T>::xd_ticket());
  const auto& participating_velocities_cache_entry = manager->DeclareCacheEntry(
      "participating velocities for all bodies",
      systems::ValueProducer(this,
                             &DeformableDriver<T>::CalcParticipatingVelocities),
      constraint_participation_and_xd_tickets);
  cache_indexes_.participating_velocities =
      participating_velocities_cache_entry.cache_index();

  /* Input ports are needed for the calculation of the free motion velocities
   (via user defined external forces). */
  auto participarting_free_motion_velocities_tickets =
      constraint_participation_and_xd_tickets;
  participarting_free_motion_velocities_tickets.insert(
      systems::System<T>::all_input_ports_ticket());
  const auto& participating_free_motion_velocities_cache_entry =
      manager->DeclareCacheEntry(
          fmt::format("participating free motion velocities for all bodies"),
          systems::ValueProducer(
              this,
              &DeformableDriver<T>::CalcParticipatingFreeMotionVelocities),
          constraint_participation_and_xd_tickets);
  cache_indexes_.participating_free_motion_velocities =
      participating_free_motion_velocities_cache_entry.cache_index();
}

template <typename T>
void DeformableDriver<T>::AppendLinearDynamicsMatrix(
    const systems::Context<T>& context, std::vector<MatrixX<T>>* A) const {
  DRAKE_DEMAND(A != nullptr);
  const int num_bodies = deformable_model_->num_bodies();
  for (DeformableBodyIndex index(0); index < num_bodies; ++index) {
    const DeformableBodyId body_id = deformable_model_->GetBodyId(index);
    if (!deformable_model_->is_enabled(body_id, context)) {
      A->push_back(MatrixX<T>::Zero(0, 0));
      continue;
    }
    const SchurComplement& schur_complement =
        EvalFreeMotionTangentMatrixSchurComplement(context, index);
    /* The schur complement is of the tangent matrix of the force balance
     whereas the linear dynamics matrix requires the tangent matrix of the
     momentum balance. Hence, we scale by dt here. */
    A->push_back(schur_complement.get_D_complement() *
                 manager_->plant().time_step());
  }
}

template <typename T>
typename DeformableDriver<T>::ContactData
DeformableDriver<T>::ComputeContactDataForDeformable(
    const systems::Context<T>& context,
    const geometry::internal::DeformableContactSurface<T>& surface,
    bool is_A) const {
  const geometry::GeometryId geometry_id =
      is_A ? surface.id_A() : surface.id_B();
  const DeformableBodyIndex body_index = deformable_model_->GetBodyIndex(
      deformable_model_->GetBodyId(geometry_id));
  const ContactParticipation& participation =
      EvalConstraintParticipation(context, body_index);
  const PartialPermutation& vertex_permutation =
      EvalVertexPermutation(context, geometry_id);
  const VectorX<T>& deformable_participating_v0 =
      EvalParticipatingVelocities(context);
  const Multiplexer<T>& mux = EvalParticipatingVelocityMultiplexer(context);
  const Eigen::Ref<const VectorX<T>> body_participating_v0 =
      mux.Demultiplex(deformable_participating_v0, body_index);
  // TODO(xuchenhan-tri): This should be pre-computed and cached instead of
  // recomputed for every deformable body in contact at every time step.
  /* Retrieve the boundary condition information of the body to determine
   which columns for the jacobian need to be zeroed out later. */
  const DeformableBodyId body_id = deformable_model_->GetBodyId(geometry_id);
  const FemModel<T>& fem_model = deformable_model_->GetFemModel(body_id);
  const DirichletBoundaryCondition<T>& bc =
      fem_model.dirichlet_boundary_condition();
  /* The number of boundary conditions added to each vertex. */
  std::vector<int> num_bcs(fem_model.num_nodes(), 0);
  for (const auto& [node_index, node_state] : bc.index_to_boundary_state()) {
    /* Note that we currently only allow zero boundary conditions. */
    DRAKE_DEMAND(node_state.v == Vector3<T>::Zero());
    DRAKE_DEMAND(node_state.a == Vector3<T>::Zero());
    ++num_bcs[node_index];
  }

  ContactData result;
  // For deformable objects, we use the centroid of the contact surface as the
  // relative-to point in the configuration.
  result.p_WG = surface.contact_mesh_W().centroid();
  // TODO(xuchenhan-tri): Currently deformable bodies don't have names. When
  // they do get names upon registration (in DeformableModel), update its
  // body name here.
  result.name = fmt::format("deformable id {}", geometry_id);
  result.v_WGc.reserve(surface.num_contact_points());
  result.jacobian.reserve(surface.num_contact_points());

  /* The Jacobian block triplets to be filled in. There are at most 4 nonzero
   blocks. */
  std::vector<typename Block3x3SparseMatrix<T>::Triplet> triplets;
  triplets.reserve(4);
  bool is_deformable_rigid = !surface.is_B_deformable();
  for (int i = 0; i < surface.num_contact_points(); ++i) {
    /* The contact Jacobian (w.r.t. v) of the velocity of the point affixed to
     the geometry that coincides with the contact point C in the world frame,
     expressed in the contact frame C. We scale it by -1 if the body corresponds
     to body A in contact to get the correct sign. */
    const double scale = is_A ? -1.0 : 1.0;
    Block3x3SparseMatrix<T> scaled_Jv_v_WGc_C(
        /* block rows */ 1,
        /* block columns */ participation.num_vertices_in_contact());
    Vector4<int> participating_vertices = Vector4<int>::Constant(-1);
    Vector4<T> barycentric_weights = Vector4<T>::Zero();
    if (is_deformable_rigid) {
      DRAKE_DEMAND(is_A);
      participating_vertices.head<3>() =
          surface.tri_contact_vertex_indexes_A()[i];
      barycentric_weights.template head<3>() =
          surface.tri_barycentric_coordinates_A()[i];
    } else {
      participating_vertices = is_A ? surface.tet_contact_vertex_indexes_A()[i]
                                    : surface.contact_vertex_indexes_B()[i];
      barycentric_weights = is_A ? surface.tet_barycentric_coordinates_A()[i]
                                 : surface.barycentric_coordinates_B()[i];
    }
    Vector3<T> v_WGc = Vector3<T>::Zero();
    triplets.clear();
    for (int v = 0; v < 4; ++v) {
      if (participating_vertices(v) < 0) {
        DRAKE_DEMAND(barycentric_weights(v) == 0.0);
        continue;
      }
      const bool vertex_under_bc = num_bcs[participating_vertices(v)] > 0;
      /* Map indexes to the permuted domain. */
      const int permuted_vertex =
          vertex_permutation.permuted_index(participating_vertices(v));
      if (!vertex_under_bc) {
        /* v_WAc = (b₀ * v₀ + b₁ * v₁ + b₂ * v₂ + b₃ * v₃) where v₀, v₁, v₂,
         v₃ are the velocities of the vertices forming the tetrahedron
         containing the contact point and the b's are their corresponding
         barycentric weights. */
        triplets.emplace_back(0, permuted_vertex,
                              scale * barycentric_weights(v) *
                                  surface.R_WCs()[i].matrix().transpose());
        v_WGc += barycentric_weights(v) *
                 body_participating_v0.template segment<3>(3 * permuted_vertex);
      }
      /* If the vertex is under bc, the corresponding jacobian block is zero
       because the vertex doesn't contribute to the contact velocity. */
    }
    scaled_Jv_v_WGc_C.SetFromTriplets(triplets);
    result.v_WGc.emplace_back(v_WGc);
    const TreeIndex tree_index(manager_->internal_tree().forest().num_trees() +
                               body_index);
    result.jacobian.emplace_back(tree_index,
                                 MatrixBlock<T>(std::move(scaled_Jv_v_WGc_C)));
  }
  return result;
}

template <typename T>
typename DeformableDriver<T>::ContactData
DeformableDriver<T>::ComputeContactDataForRigid(
    const systems::Context<T>& context,
    const geometry::internal::DeformableContactSurface<T>& surface) const {
  ContactData result;
  /* Rigid geometry is guaranteed to be body B in a deformable rigid contact. */
  const geometry::GeometryId geometry_id = surface.id_B();
  const BodyIndex body_index = manager_->FindBodyByGeometryId(geometry_id);
  const RigidBody<T>& rigid_body = manager_->plant().get_body(body_index);
  result.name = rigid_body.name();
  /* For rigid body, we use the origin of the body frame as the relative-to
   point. */
  result.p_WG =
      manager_->plant()
          .EvalBodyPoseInWorld(context, manager_->plant().get_body(body_index))
          .translation();
  if (body_index == world_index()) return result;

  const SpanningForest& forest = manager_->internal_tree().forest();
  const TreeIndex tree_index = forest.link_to_tree_index(body_index);
  const SpanningForest::Tree& tree = forest.trees(tree_index);
  const int tree_nv = tree.nv();
  /* If the body is welded to world, then everything is trivially zero (as
   indicated by empty jacobian and velocity vectors). */
  if (tree_nv == 0) return result;

  const Eigen::VectorBlock<const VectorX<T>> rigid_v0 =
      manager_->plant().GetVelocities(context);
  const Frame<T>& frame_W = manager_->plant().world_frame();
  const int nv = manager_->plant().num_velocities();
  Matrix3X<T> Jv_v_WGc_W(3, nv);
  for (int i = 0; i < surface.num_contact_points(); ++i) {
    // TODO(xuchenhan-tri): The computation of the contact Jacobian for all
    // contact points associated with this contact surface can be done in a
    // single pass down the kinematic path to world.
    const Vector3<T>& p_WC = surface.contact_points_W()[i];
    manager_->internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, rigid_body.body_frame(), frame_W,
        p_WC, frame_W, frame_W, &Jv_v_WGc_W);
    result.v_WGc.emplace_back(Jv_v_WGc_W * rigid_v0);
    Matrix3X<T> J = surface.R_WCs()[i].matrix().transpose() *
                    Jv_v_WGc_W.middleCols(tree.v_start(), tree_nv);
    result.jacobian.emplace_back(tree_index, MatrixBlock<T>(std::move(J)));
  }
  return result;
}

template <typename T>
void DeformableDriver<T>::AppendDiscreteContactPairs(
    const systems::Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* result) const {
  DRAKE_DEMAND(result != nullptr);

  /* Since v_AcBc_W = v_WBc - v_WAc the relative velocity Jacobian will be:
     Jv_v_AcBc_W = Jv_v_WBc_W - Jv_v_WAc_W.
   That is the relative velocity at C is v_AcBc_W = Jv_v_AcBc_W * v.
   Finally Jv_v_AcBc_C = R_WC.transpose() * Jv_v_AcBc_W.
   The set of dofs for body A and body B are mutually exclusive. */

  const geometry::SceneGraphInspector<T>& inspector =
      manager_->plant().EvalSceneGraphInspector(context);
  const DeformableContact<T>& deformable_contact =
      EvalDeformableContact(context);
  const std::vector<DeformableContactSurface<T>>& contact_surfaces =
      deformable_contact.contact_surfaces();

  for (int surface_index = 0; surface_index < ssize(contact_surfaces);
       ++surface_index) {
    const DeformableContactSurface<T>& surface =
        contact_surfaces[surface_index];
    /* Geometry A is always deformable, so this is deformable vs. deformable
       contact iff geometry B is deformable. */
    const bool is_deformable_vs_deformable = surface.is_B_deformable();
    /* Skip this surface if either body in contact is a disabled deformable
     body. */
    const DeformableBodyId body_id_A =
        deformable_model_->GetBodyId(surface.id_A());
    const bool is_A_disabled =
        !deformable_model_->is_enabled(body_id_A, context);
    const bool is_B_disabled =
        surface.is_B_deformable() &&
        !deformable_model_->is_enabled(
            deformable_model_->GetBodyId(surface.id_B()), context);
    if (is_A_disabled || is_B_disabled) {
      continue;
    }
    /* Write the contact jacobian and velocity for all contact points for body
     A. */
    ContactData contact_data_A =
        ComputeContactDataForDeformable(context, surface, true);
    const Vector3<T>& p_WA = contact_data_A.p_WG;

    ContactData contact_data_B =
        surface.is_B_deformable()
            ? ComputeContactDataForDeformable(context, surface, false)
            : ComputeContactDataForRigid(context, surface);
    const Vector3<T>& p_WB = contact_data_B.p_WG;

    const GeometryId id_A = surface.id_A();
    const GeometryId id_B = surface.id_B();
    const std::string& name_A = inspector.GetName(id_A);
    const std::string& name_B = inspector.GetName(id_B);
    /* Body A is guaranteed to be deformable. */
    const int body_index_A =
        deformable_model_->GetBodyIndex(deformable_model_->GetBodyId(id_A));
    /* Body B may be rigid or deformable. We first get its body index. */
    const int body_index_B =
        surface.is_B_deformable()
            ? static_cast<int>(deformable_model_->GetBodyIndex(
                  deformable_model_->GetBodyId(id_B)))
            : static_cast<int>(manager_->FindBodyByGeometryId(id_B));
    /* By convention, the object index of a rigid body is just its body index.
     All deformable bodies come after all rigid bodies, and the object index of
     a deformable body is its deformable body index + the number of rigid bodies
     in the plant. */
    const int object_A =
        body_index_A + manager_->plant().num_bodies();  // Deformable body.
    const int object_B =
        surface.is_B_deformable()
            ? body_index_B + manager_->plant().num_bodies()  // Deformable body.
            : body_index_B;                                  // Rigid body.

    /* We reuse `jacobian_blocks` for the Jacobian blocks for each contact
     point and clear the vector repeatedly in the loop over the contact points.
    */
    std::vector<typename DiscreteContactPair<T>::JacobianTreeBlock>
        jacobian_blocks;
    for (int i = 0; i < surface.num_contact_points(); ++i) {
      if (!is_deformable_vs_deformable && surface.is_element_inverted()[i]) {
        /* For deformable rigid contact, we don't register a contact point for
        deformable surface elements that belong to an inverted tetrahdron.
        Registering such contact points can cause persistent inversion artifacts
        that are impossible to recover. By not registering a contact point, we
        admit transient artifacts (objects suddenly break contact), but in
        practice, we find that's better than persistent, irrecoverable
        inversion. */
        continue;
      }

      jacobian_blocks.clear();
      const Vector3<T>& v_WAc = contact_data_A.v_WGc[i];
      jacobian_blocks.push_back(std::move(contact_data_A.jacobian[i]));

      Vector3<T> v_WBc = Vector3<T>::Zero();
      /* Empty contact data indicates that the body is welded and we don't need
       to record the Jacobian. */
      if (!contact_data_B.jacobian.empty()) {
        v_WBc = contact_data_B.v_WGc[i];
        jacobian_blocks.push_back(std::move(contact_data_B.jacobian[i]));
      }

      const Vector3<T>& p_WC = surface.contact_points_W()[i];
      // Contact point position relative to object A and B.
      const Vector3<T> p_AC_W = p_WC - p_WA;
      const Vector3<T> p_BC_W = p_WC - p_WB;

      /* We set a large stiffness for the deformable body to approximate rigid
       contact. We choose a large constant C with units of [Pa/m] and then scale
       the constant C by the area of the contact polygon to compute an effective
       stiffness k that has units of [N/m]. We choose the value of C from the
       following approximation: Consider a unit cube with side length L in
       equilibrium on the ground under gravity. The contact force is equal to
       k * ϕ where ϕ is the penetration distance. The contact force balances
       gravity and we have

         kϕ  = CL²ϕ = GρL³,

       which gives ϕ = GρL / C.
       We choose a large C = 1e8 Pa/m so that for ρ = 1000 kg/m³ and
       G = 10 m/s², we get ϕ = 1e-4 * L, or 0.01 mm for a 10 cm cube with
       density of water, a reasonably small penetration. */
      const T C = 1e8;

      const Vector3<T>& nhat_BA_W = surface.nhats_W()[i];
      const T Ae = surface.contact_mesh_W().area(i);
      std::optional<T> rigid_g;
      if (!is_deformable_vs_deformable) {
        /* One dimensional pressure gradient (in Pa/m), only used in deformable
         vs. rigid contact, following the notation in [Masterjohn, 2022].
         [Masterjohn, 2022] Velocity Level Approximation of Pressure Field
         Contact Patches.
         We Filter out negative directional pressure derivatives (separating
         contact) and tiny contact polygons (to avoid numerical issues).
         Unlike [Masterjohn, 2022], the pressure gradient is positive in the
         direction "into" the rigid body. Therefore, we need a negative sign. */
        rigid_g = -surface.pressure_gradients_W()[i].dot(nhat_BA_W);

        /* We found that geometry queries might repor tiny triangles (consider
         for instance an initial condition that perfectly places an object at
         zero distance from the ground.) While the area of zero sized triangles
         is not a problem by itself, the badly computed normal on these
         triangles leads to problems when computing the contact Jacobians (since
         we need to obtain an orthonormal basis based on that normal). We
         therefore ignore tiny triangles with areas below a certain arbitrary
         threshold (1e-14 m²). We also ignore pressure gradients below a certain
         threshold (1e-14 Pa/m) following the hydroelastic treatment. */
        if (rigid_g.value() < 1e-14 || Ae < 1e-14) {
          continue;
        }
      }

      const T deformable_k = Ae * C;
      const double default_dissipation =
          manager_->default_contact_dissipation();
      const auto [k, phi0, fn0, d] = [&]() {
        if (is_deformable_vs_deformable) {
          const T deformable_phi0 = surface.signed_distances()[i];
          const T deformable_fn0 = -deformable_k * deformable_phi0;
          const T deformable_d = GetCombinedHuntCrossleyDissipation(
              id_A, id_B, deformable_k, deformable_k, default_dissipation,
              inspector);
          return std::make_tuple(deformable_k, deformable_phi0, deformable_fn0,
                                 deformable_d);
        } else {
          DRAKE_ASSERT(rigid_g.has_value());
          /* The one dimensional gradient for the deformable geometry based on
           the "box-on-ground" estimate above. */
          const T deformable_g = C;
          /* Combined "effective" one dimensional gradient. */
          const T g = 1.0 / (1.0 / deformable_g + 1.0 / rigid_g.value());
          const T rigid_k = Ae * g;
          const T rigid_phi0 = -surface.pressures()[i] / g;
          const T rigid_fn0 = Ae * surface.pressures()[i];
          const T rigid_d = GetCombinedHuntCrossleyDissipation(
              id_A, id_B, deformable_k, rigid_k, default_dissipation,
              inspector);
          return std::make_tuple(rigid_k, rigid_phi0, rigid_fn0, rigid_d);
        }
      }();

      const double default_dissipation_time_constant = 0.1;
      const T tau = GetCombinedDissipationTimeConstant(
          id_A, id_B, default_dissipation_time_constant, name_A, name_B,
          inspector);
      const double mu =
          GetCombinedDynamicCoulombFriction(id_A, id_B, inspector);

      /* Contact solver assumes the normal points from A to B whereas the
       surface's normal points from B to A. */
      const Vector3<T> nhat_AB_W = -surface.nhats_W()[i];
      const math::RotationMatrix<T>& R_WC = surface.R_WCs()[i];
      /* The normal (scalar) component of the contact velocity in the contact
       frame. */
      const T v_AcBc_Cz = nhat_AB_W.dot(v_WBc - v_WAc);
      DiscreteContactPair<T> contact_pair{
          .jacobian = std::move(jacobian_blocks),
          .id_A = id_A,
          .object_A = object_A,
          .id_B = id_B,
          .object_B = object_B,
          .R_WC = R_WC,
          .p_WC = p_WC,
          .p_ApC_W = p_AC_W,
          .p_BqC_W = p_BC_W,
          .nhat_BA_W = nhat_BA_W,
          .phi0 = phi0,
          .vn0 = v_AcBc_Cz,
          .fn0 = fn0,
          .stiffness = k,
          .damping = d,
          .dissipation_time_scale = tau,
          .friction_coefficient = mu,
          .surface_index = surface_index,
          .face_index = i};
      result->AppendDeformableData(std::move(contact_pair));
    }
  }
}

template <typename T>
void DeformableDriver<T>::AppendDeformableRigidFixedConstraintKinematics(
    const systems::Context<T>& context,
    std::vector<FixedConstraintKinematics<T>>* result) const {
  DRAKE_DEMAND(result != nullptr);
  /* For a fixed constraint between a point p on a deformable body A and a point
   q on a rigid body B, we define the velocity difference as

     v_ApBq_W = v_WBq - v_WAp.

   The relative velocity Jacobian is then:

     Jv_v_W_ApBq = Jv_v_WBq - Jv_v_WAp.

   We encode the jacobian as two jacobian blocks, each with its own sparsity
   pattern, by exploiting the fact that the set of DoFs for deformable bodies
   and rigid bodies are mutually exclusive, and Jv_v_WAp = 0 for rigid dofs
   and Jv_v_WBq = 0 for deformable dofs. */
  const int nv = manager_->plant().num_velocities();
  const SpanningForest& forest = manager_->internal_tree().forest();
  const auto& configurations =
      manager_->plant()
          .get_deformable_body_configuration_output_port()
          .template Eval<geometry::GeometryConfigurationVector<T>>(context);
  for (DeformableBodyIndex index(0); index < deformable_model_->num_bodies();
       ++index) {
    const DeformableBody<T>& body = deformable_model_->GetBody(index);
    if (!body.is_enabled(context) || !body.has_fixed_constraint()) {
      continue;
    }

    const FemModel<T>& fem_model = body.fem_model();
    const DirichletBoundaryCondition<T>& bc =
        fem_model.dirichlet_boundary_condition();

    /* Returns true iff for the deformable body, the given vertex index is under
     boundary condition. */
    auto is_under_bc = [&](int vertex_index) {
      return bc.index_to_boundary_state().count(
                 multibody::fem::FemNodeIndex(vertex_index)) > 0;
    };

    const ContactParticipation& participation =
        EvalConstraintParticipation(context, index);
    // Each deformable body forms its own clique and are indexed in inceasing
    // DeformableBodyIndex order and placed after all rigid cliques.
    const TreeIndex clique_index_A(forest.num_trees() + index);
    const GeometryId geometry_id = body.geometry_id();
    const PartialPermutation& vertex_permutation =
        EvalVertexPermutation(context, geometry_id);
    const VectorX<T>& p_WVs = configurations.value(geometry_id);

    for (const DeformableRigidFixedConstraintSpec& spec :
         body.fixed_constraint_specs()) {
      const int num_vertices_in_constraint = ssize(spec.vertices);
      /* The Jacobian block for the deformable body A. */
      Block3x3SparseMatrix<T> negative_Jv_v_WAp(
          num_vertices_in_constraint, participation.num_vertices_in_contact());
      std::vector<typename Block3x3SparseMatrix<T>::Triplet> jacobian_triplets;
      jacobian_triplets.reserve(num_vertices_in_constraint);
      /* The Jacobian block for the rigid body B. */
      const BodyIndex index_B = spec.body_B;
      const RigidBody<T>& rigid_body = manager_->plant().get_body(index_B);
      const math::RigidTransform<T>& X_WB =
          manager_->plant().EvalBodyPoseInWorld(context, rigid_body);
      /* The positions of the anchor points on the rigid body in world frame. */
      VectorX<T> p_WQs(3 * num_vertices_in_constraint);
      /* The positions of the anchor points on the rigid body in rigid body's
       frame. */
      VectorX<T> p_BQs(3 * num_vertices_in_constraint);
      for (int v = 0; v < num_vertices_in_constraint; ++v) {
        /* The Jacobian for the deformable body is identity for a free vertex
         and zero otherwise. */
        if (!is_under_bc(spec.vertices[v])) {
          const int permuted_vertex_index =
              vertex_permutation.permuted_index(spec.vertices[v]);
          jacobian_triplets.emplace_back(v, permuted_vertex_index,
                                         -Matrix3<T>::Identity());
        }
        p_BQs.template segment<3>(3 * v) = spec.p_BQs[v].cast<T>();
        p_WQs.template segment<3>(3 * v) = X_WB * spec.p_BQs[v].cast<T>();
      }
      negative_Jv_v_WAp.SetFromTriplets(jacobian_triplets);
      MatrixBlock<T> jacobian_block_A(std::move(negative_Jv_v_WAp));

      /* Positions of fixed vertices of the deformable body in the deformable
       body's frame which is always assumed to be the world frame. */
      VectorX<T> p_WPs(3 * spec.vertices.size());
      for (int v = 0; v < ssize(spec.vertices); ++v) {
        p_WPs.template segment<3>(3 * v) =
            p_WVs.template segment<3>(3 * spec.vertices[v]);
      }
      VectorX<T> p_PQs_W = p_WQs - p_WPs;
      /* By convention, deformable bodies are assigned object indexes after all
       rigid bodies. */
      const int object_A =
          index + manager_->plant().num_bodies();  // Deformable body.
      const int object_B = index_B;                // Rigid body.

      const TreeIndex tree_index = forest.link_to_tree_index(index_B);
      if (tree_index.is_valid()) {
        /* Rigid body is not welded to world. */
        const int clique_index_B = tree_index;
        const Frame<T>& frame_W = manager_->plant().world_frame();
        MatrixX<T> Jv_v_WBq(3 * num_vertices_in_constraint, nv);
        manager_->internal_tree().CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable::kV, rigid_body.body_frame(), frame_W,
            Eigen::Map<const Matrix3X<T>>(p_WQs.data(), 3, p_WQs.size() / 3),
            frame_W, frame_W, &Jv_v_WBq);
        const SpanningForest::Tree& tree = forest.trees(tree_index);
        MatrixBlock<T> jacobian_block_B(
            Jv_v_WBq.middleCols(tree.v_start(), tree.nv()));
        SapConstraintJacobian<T> J(clique_index_A, std::move(jacobian_block_A),
                                   clique_index_B, std::move(jacobian_block_B));
        result->emplace_back(object_A, std::move(p_WPs), object_B,
                             std::move(p_BQs), std::move(p_PQs_W),
                             std::move(J));
      } else {
        /* Rigid body is welded to world. */
        SapConstraintJacobian<T> J(clique_index_A, std::move(jacobian_block_A));
        result->emplace_back(object_A, std::move(p_WPs), std::move(p_PQs_W),
                             std::move(J));
      }
    }
  }
}

template <typename T>
void DeformableDriver<T>::CalcDeformableContactInfo(
    const DeformableContact<T>& deformable_contact,
    const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs,
    const ContactSolverResults<T>& solver_results,
    std::vector<DeformableContactInfo<T>>* contact_info) const {
  DRAKE_DEMAND(contact_info != nullptr);

  const std::vector<DeformableContactSurface<T>>& contact_surfaces =
      deformable_contact.contact_surfaces();
  const int num_surfaces = contact_surfaces.size();

  contact_info->clear();
  contact_info->reserve(num_surfaces);

  const VectorX<T>& fn = solver_results.fn;
  const VectorX<T>& ft = solver_results.ft;
  const VectorX<T>& vt = solver_results.vt;
  const VectorX<T>& vn = solver_results.vn;

  const int num_contacts = contact_pairs.size();
  DRAKE_DEMAND(fn.size() == num_contacts);
  DRAKE_DEMAND(ft.size() == 2 * num_contacts);
  DRAKE_DEMAND(vn.size() == num_contacts);
  DRAKE_DEMAND(vt.size() == 2 * num_contacts);

  /* The spatial force on the deformable body A at the centroid of the contact
   patch Ao, expressed in the world frame. */
  std::vector<SpatialForce<T>> F_Ao_W_per_surface(num_surfaces,
                                                  SpatialForce<T>::Zero());
  std::vector<std::vector<DeformableContactPointData<T>>> contact_point_data(
      num_surfaces);
  for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
    contact_point_data[surface_index].reserve(
        contact_surfaces[surface_index].num_contact_points());
  }

  for (int icontact = contact_pairs.deformable_contact_start();
       icontact < contact_pairs.deformable_contact_start() +
                      contact_pairs.num_deformable_contacts();
       ++icontact) {
    const DiscreteContactPair<T>& pair = contact_pairs[icontact];
    /* Contact point C. */
    const Vector3<T>& p_WC = pair.p_WC;
    const math::RotationMatrix<T>& R_WC = pair.R_WC;

    /* Contact forces applied on B at contact point point C expressed in the
     contact frame. */
    const Vector3<T> f_Bc_C(ft(2 * icontact), ft(2 * icontact + 1),
                            fn(icontact));
    /* Contact force applied on A at contact point C in the world frame. */
    const Vector3<T> f_Ac_W = -(R_WC * f_Bc_C);

    /* All deformable discrete contact pair has `surface_index` attribute. */
    DRAKE_DEMAND(pair.surface_index.has_value());
    const int surface_index = pair.surface_index.value();
    const auto& s = contact_surfaces[surface_index];
    /* Surface's centroid point O. */
    const Vector3<T>& p_WO = s.contact_mesh_W().centroid();

    /* Accumulate spatial force at the centroid of the contact patch for the
     corresponding contact surface. */
    const Vector3<T> p_CO_W = p_WO - p_WC;
    const SpatialForce<T> Fc_Ao_W =
        SpatialForce<T>(Vector3<T>::Zero(), f_Ac_W).Shift(p_CO_W);
    F_Ao_W_per_surface[surface_index] += Fc_Ao_W;

    /* Velocity of Ac relative to Bc in the tangent direction. */
    /* N.B. Computation of contact kinematics uses the convention of computing
     J_AcBc_C and thus J_AcBc_C * v = v_AcBc_W (i.e. it computes the relative
     velocity of Bc with respect to Ac). Thus we flip the sign here for the
     convention used by DeformableContactPointData, which measures the relative
     velocity of Ac w.r.t. Bc). */
    const Vector3<T> vt_BcAc_C(-vt(2 * icontact), -vt(2 * icontact + 1), 0);
    const Vector3<T> vt_BcAc_W = R_WC * vt_BcAc_C;

    /* Traction vector applied to body A at point Ac (Ac and Bc are coincident)
     expressed in the world frame. */
    DRAKE_DEMAND(pair.face_index.has_value());
    const int face_index = pair.face_index.value();
    const Vector3<T> traction_Ac_W =
        f_Ac_W / s.contact_mesh_W().area(face_index);

    contact_point_data[surface_index].emplace_back(p_WC, face_index, vt_BcAc_W,
                                                   traction_Ac_W);
  }
  for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
    const DeformableContactSurface<T>& surface =
        contact_surfaces[surface_index];
    // TODO(xuchenhan-tri): consider avoid making a copy of the contact mesh
    // every time DeformableContactInfo is computed.
    contact_info->emplace_back(surface.id_A(), surface.id_B(),
                               contact_surfaces[surface_index].contact_mesh_W(),
                               F_Ao_W_per_surface[surface_index]);
  }
}

template <typename T>
void DeformableDriver<T>::CalcDiscreteStates(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* next_states) const {
  const int num_bodies = deformable_model_->num_bodies();
  for (DeformableBodyIndex index(0); index < num_bodies; ++index) {
    const FemState<T>& next_fem_state = EvalNextFemState(context, index);
    const int num_dofs = next_fem_state.num_dofs();
    // Update the discrete values.
    VectorX<T> discrete_value(num_dofs * 3);
    discrete_value.head(num_dofs) = next_fem_state.GetPositions();
    discrete_value.segment(num_dofs, num_dofs) = next_fem_state.GetVelocities();
    discrete_value.tail(num_dofs) = next_fem_state.GetAccelerations();
    const DeformableBodyId id = deformable_model_->GetBodyId(index);
    next_states->set_value(deformable_model_->GetDiscreteStateIndex(id),
                           discrete_value);
  }
}

template <typename T>
Multiplexer<T>::Multiplexer(std::vector<int> sizes) : sizes_(std::move(sizes)) {
  DRAKE_THROW_UNLESS(!sizes_.empty());
  DRAKE_THROW_UNLESS(sizes_[0] >= 0);
  offsets_.resize(num_vectors());
  offsets_[0] = 0;
  for (int i = 1; i < num_vectors(); ++i) {
    DRAKE_THROW_UNLESS(sizes_[i] >= 0);
    offsets_[i] = offsets_[i - 1] + sizes_[i - 1];
  }
  num_entries_ = std::accumulate(sizes_.begin(), sizes_.end(), 0);
}

template <typename T>
VectorX<T> Multiplexer<T>::Multiplex(std::vector<VectorX<T>>&& inputs) const {
  VectorX<T> result(num_entries_);
  DRAKE_THROW_UNLESS(static_cast<int>(inputs.size()) == num_vectors());
  for (int i = 0; i < num_vectors(); ++i) {
    DRAKE_THROW_UNLESS(sizes_[i] == inputs[i].size());
    /* We move inputs[i] into the block vector so that (hopefully) the move
     semantic is applied to individual entries. This may be significant when
     T != double. */
    result.segment(offsets_[i], sizes_[i]) = std::move(inputs[i]);
  }
  return result;
}

template <typename T>
Eigen::Ref<const VectorX<T>> Multiplexer<T>::Demultiplex(
    const Eigen::Ref<const VectorX<T>>& input, int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < num_vectors());
  DRAKE_THROW_UNLESS(input.size() == num_entries_);
  /* We prefer Eigen::Ref<const VectorX<T>> to Eigen::VectorBlock<const
   Eigen::Ref<const VectorX<T>>> for readability. */
  return input.segment(offsets_[index], sizes_[index]);
}

template <typename T>
Eigen::Ref<VectorX<T>> Multiplexer<T>::Demultiplex(EigenPtr<VectorX<T>> input,
                                                   int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < num_vectors());
  DRAKE_THROW_UNLESS(input->size() == num_entries_);
  /* We prefer Eigen::Ref<VectorX<T>> to
   Eigen::VectorBlock<Eigen::Ref<VectorX<T>>> for readability and encapsulation
   of the implementation of EigenPtr. */
  return input->segment(offsets_[index], sizes_[index]);
}

template <typename T>
void DeformableDriver<T>::CalcFemState(const Context<T>& context,
                                       DeformableBodyIndex index,
                                       FemState<T>* fem_state) const {
  const DeformableBodyId id = deformable_model_->GetBodyId(index);
  const systems::BasicVector<T>& discrete_state =
      context.get_discrete_state().get_vector(
          deformable_model_->GetDiscreteStateIndex(id));
  const VectorX<T>& discrete_value = discrete_state.value();
  DRAKE_DEMAND(discrete_value.size() % 3 == 0);
  const int num_dofs = discrete_value.size() / 3;
  const auto& q = discrete_value.head(num_dofs);
  const auto& qdot = discrete_value.segment(num_dofs, num_dofs);
  const auto& qddot = discrete_value.tail(num_dofs);
  fem_state->SetPositions(q);
  fem_state->SetVelocities(qdot);
  fem_state->SetAccelerations(qddot);
}

template <typename T>
const FemState<T>& DeformableDriver<T>::EvalFemState(
    const Context<T>& context, DeformableBodyIndex index) const {
  const DeformableBody<T>& body = deformable_model_->GetBody(index);
  return manager_->plant()
      .get_cache_entry(body.fem_state_cache_index())
      .template Eval<FemState<T>>(context);
}

template <typename T>
void DeformableDriver<T>::CalcFreeMotionFemSolver(
    const systems::Context<T>& context, DeformableBodyIndex index,
    FemSolver<T>* fem_solver) const {
  const DeformableBodyId body_id = deformable_model_->GetBodyId(index);
  const GeometryId geometry_id = deformable_model_->GetGeometryId(body_id);
  const FemState<T>& fem_state = EvalFemState(context, index);
  if (!deformable_model_->is_enabled(body_id, context)) {
    fem_solver->SetNextFemState(fem_state);
    return;
  }
  /* Write the non-participating vertices. */
  std::unordered_set<int> nonparticipating_vertices;
  const PartialPermutation& permutation =
      EvalVertexPermutation(context, geometry_id);
  DRAKE_DEMAND(3 * permutation.domain_size() == fem_state.num_dofs());
  for (int v = 0; v < permutation.domain_size(); ++v) {
    if (!permutation.participates(v)) {
      nonparticipating_vertices.insert(v);
    }
  }
  /* Collect all external forces affecting this body and store them into the
   FemPlantData for the associated FEM model. */
  const fem::FemPlantData<T> plant_data{
      context, deformable_model_->GetExternalForces(body_id)};
  fem_solver->AdvanceOneTimeStep(fem_state, plant_data,
                                 nonparticipating_vertices);
}

template <typename T>
const FemSolver<T>& DeformableDriver<T>::EvalFreeMotionFemSolver(
    const systems::Context<T>& context, DeformableBodyIndex index) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.fem_solvers.at(index))
      .template Eval<FemSolver<T>>(context);
}

template <typename T>
const FemState<T>& DeformableDriver<T>::EvalFreeMotionFemState(
    const systems::Context<T>& context, DeformableBodyIndex index) const {
  const FemSolver<T>& fem_solver = EvalFreeMotionFemSolver(context, index);
  return fem_solver.next_fem_state();
}

template <typename T>
const SchurComplement&
DeformableDriver<T>::EvalFreeMotionTangentMatrixSchurComplement(
    const systems::Context<T>& context, DeformableBodyIndex index) const {
  const FemSolver<T>& fem_solver = EvalFreeMotionFemSolver(context, index);
  return fem_solver.next_schur_complement();
}

template <typename T>
void DeformableDriver<T>::CalcNextFemState(const systems::Context<T>& context,
                                           DeformableBodyIndex index,
                                           FemState<T>* next_fem_state) const {
  const ContactParticipation& participation =
      EvalConstraintParticipation(context, index);
  if (participation.num_vertices_in_contact() == 0) {
    /* The next states are the free motion states if no vertex of the
     deformable body participates in contact. */
    const FemState<T>& free_motion_state =
        EvalFreeMotionFemState(context, index);
    next_fem_state->SetPositions(free_motion_state.GetPositions());
    next_fem_state->SetVelocities(free_motion_state.GetVelocities());
    next_fem_state->SetAccelerations(free_motion_state.GetAccelerations());
  } else {
    const ContactSolverResults<T>& results =
        manager_->EvalContactSolverResults(context);
    /* Get the next time step velocities and free motion velocities for *all*
     participating deformable dofs (belonging to *all* deformable bodies). */
    const int num_participating_dofs =
        results.v_next.size() - manager_->plant().num_velocities();
    const auto participating_v_next =
        results.v_next.tail(num_participating_dofs);
    const VectorX<T>& participating_v_star =
        EvalParticipatingFreeMotionVelocities(context);
    /* The next time step velocities and free motion velocities for *this*
     body. */
    const Multiplexer<T>& mux = EvalParticipatingVelocityMultiplexer(context);
    const Eigen::Ref<const VectorX<T>> body_participating_v_star =
        mux.Demultiplex(participating_v_star, index);
    const Eigen::Ref<const VectorX<T>> body_participating_v_next =
        mux.Demultiplex(participating_v_next, index);
    const VectorX<T> body_participating_dv =
        body_participating_v_next - body_participating_v_star;
    /* Compute the value of the post-constraint non-participating
     velocities using Schur complement. */
    const SchurComplement& schur_complement =
        EvalFreeMotionTangentMatrixSchurComplement(context, index);
    const VectorX<T> body_nonparticipating_dv =
        schur_complement.SolveForX(body_participating_dv);
    /* Concatenate the participating and non-participating velocities and
     then apply the inverse permutation to put the dofs in their original
     order. */
    PartialPermutation full_velocity_permutation =
        EvalDofPermutation(context, index);
    full_velocity_permutation.ExtendToFullPermutation();
    const int num_dofs = full_velocity_permutation.domain_size();
    VectorX<T> permuted_dv(num_dofs);
    permuted_dv << body_participating_dv, body_nonparticipating_dv;
    VectorX<T> dv(num_dofs);
    full_velocity_permutation.ApplyInverse(permuted_dv, &dv);
    /* v_next = v_star + dv */
    VectorX<T>& v_next = dv;
    v_next += EvalFreeMotionFemState(context, index).GetVelocities();
    const FemState<T>& fem_state = EvalFemState(context, index);
    deformable_model_->integrator().AdvanceOneTimeStep(fem_state, v_next,
                                                       next_fem_state);
  }
}

template <typename T>
const FemState<T>& DeformableDriver<T>::EvalNextFemState(
    const systems::Context<T>& context, DeformableBodyIndex index) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.next_fem_states.at(index))
      .template Eval<FemState<T>>(context);
}

template <typename T>
void DeformableDriver<T>::CalcDeformableContact(
    const geometry::QueryObject<T>& query_object,
    DeformableContact<T>* result) const {
  /* Compute the DeformableContact from the geometry engine. */
  query_object.ComputeDeformableContact(result);

  /* Complete the result with information on constraints other than contact. */
  for (DeformableBodyIndex body_index(0);
       body_index < deformable_model_->num_bodies(); ++body_index) {
    const DeformableBody<T>& body = deformable_model_->GetBody(body_index);
    /* Add in constraints. */
    if (body.has_fixed_constraint()) {
      std::unordered_set<int> fixed_vertices;
      for (const DeformableRigidFixedConstraintSpec& spec :
           body.fixed_constraint_specs()) {
        fixed_vertices.insert(spec.vertices.begin(), spec.vertices.end());
      }
      /* Register the geometry in case it hasn't been registered because it's
       not participating in contact but is participating in other types of
       constraints. */
      GeometryId geometry_id = body.geometry_id();
      if (!result->IsRegistered(geometry_id)) {
        result->RegisterDeformableGeometry(geometry_id,
                                           body.fem_model().num_nodes());
      }
      result->Participate(geometry_id, fixed_vertices);
    }
  }
}

template <typename T>
const DeformableContact<T>& DeformableDriver<T>::EvalDeformableContact(
    const Context<T>& context) const {
  return manager_->EvalGeometryContactData(context).get().deformable;
}

template <typename T>
void DeformableDriver<T>::CalcConstraintParticipation(
    const systems::Context<T>& context, DeformableBodyIndex index,
    geometry::internal::ContactParticipation* constraint_participation) const {
  DRAKE_DEMAND(constraint_participation != nullptr);
  const DeformableBodyId body_id = deformable_model_->GetBodyId(index);
  if (!deformable_model_->is_enabled(body_id, context)) {
    const int num_vertices =
        deformable_model_->GetFemModel(body_id).num_nodes();
    *constraint_participation =
        geometry::internal::ContactParticipation(num_vertices);
    return;
  }
  const GeometryId geometry_id = deformable_model_->GetGeometryId(body_id);
  const DeformableContact<T>& contact_data = EvalDeformableContact(context);
  DRAKE_DEMAND(contact_data.IsRegistered(geometry_id));
  *constraint_participation = contact_data.contact_participation(geometry_id);
}

template <typename T>
const ContactParticipation& DeformableDriver<T>::EvalConstraintParticipation(
    const Context<T>& context, DeformableBodyIndex index) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.constraint_participations.at(index))
      .template Eval<ContactParticipation>(context);
}

template <typename T>
const PartialPermutation& DeformableDriver<T>::EvalDofPermutation(
    const Context<T>& context, DeformableBodyIndex index) const {
  const DeformableBodyId body_id = deformable_model_->GetBodyId(index);
  const GeometryId geometry_id = deformable_model_->GetGeometryId(body_id);
  return manager_->plant()
      .get_cache_entry(cache_indexes_.vertex_permutations.at(geometry_id))
      .template Eval<VertexPartialPermutation>(context)
      .dof();
}

template <typename T>
void DeformableDriver<T>::CalcPermutation(
    const Context<T>& context, GeometryId id,
    VertexPartialPermutation* result) const {
  const DeformableBodyIndex index =
      deformable_model_->GetBodyIndex(deformable_model_->GetBodyId(id));
  *result =
      EvalConstraintParticipation(context, index).CalcPartialPermutation();
}

template <typename T>
const PartialPermutation& DeformableDriver<T>::EvalVertexPermutation(
    const Context<T>& context, GeometryId id) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.vertex_permutations.at(id))
      .template Eval<VertexPartialPermutation>(context)
      .vertex();
}

template <typename T>
void DeformableDriver<T>::CalcParticipatingVelocityMultiplexer(
    const Context<T>& context, Multiplexer<T>* result) const {
  const int num_bodies = deformable_model_->num_bodies();
  std::vector<int> num_participating_dofs(num_bodies);
  for (DeformableBodyIndex i(0); i < num_bodies; ++i) {
    num_participating_dofs[i] =
        EvalDofPermutation(context, i).permuted_domain_size();
  }
  *result = Multiplexer<T>(std::move(num_participating_dofs));
}

template <typename T>
const Multiplexer<T>& DeformableDriver<T>::EvalParticipatingVelocityMultiplexer(
    const Context<T>& context) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.participating_velocity_mux)
      .template Eval<Multiplexer<T>>(context);
}

template <typename T>
void DeformableDriver<T>::CalcParticipatingVelocities(
    const Context<T>& context, VectorX<T>* result) const {
  const int num_bodies = deformable_model_->num_bodies();
  std::vector<VectorX<T>> participating_velocities(num_bodies);
  for (DeformableBodyIndex i(0); i < num_bodies; ++i) {
    const PartialPermutation& permutation = EvalDofPermutation(context, i);
    const VectorX<T>& v = EvalFemState(context, i).GetVelocities();
    participating_velocities[i].resize(permutation.permuted_domain_size());
    permutation.Apply(v, &participating_velocities[i]);
  }
  *result = EvalParticipatingVelocityMultiplexer(context).Multiplex(
      std::move(participating_velocities));
}

template <typename T>
const VectorX<T>& DeformableDriver<T>::EvalParticipatingVelocities(
    const Context<T>& context) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.participating_velocities)
      .template Eval<VectorX<T>>(context);
}

template <typename T>
void DeformableDriver<T>::CalcParticipatingFreeMotionVelocities(
    const Context<T>& context, VectorX<T>* result) const {
  DRAKE_DEMAND(result != nullptr);
  const int num_bodies = deformable_model_->num_bodies();
  std::vector<VectorX<T>> participating_v_star(num_bodies);
  for (DeformableBodyIndex i(0); i < num_bodies; ++i) {
    const PartialPermutation& permutation = EvalDofPermutation(context, i);
    const VectorX<T>& v_star =
        EvalFreeMotionFemState(context, i).GetVelocities();
    participating_v_star[i].resize(permutation.permuted_domain_size());
    permutation.Apply(v_star, &participating_v_star[i]);
  }
  // TODO(xuchenhan-tri): Consider adding a in-place version of Multiplex.
  *result = EvalParticipatingVelocityMultiplexer(context).Multiplex(
      std::move(participating_v_star));
}

template <typename T>
const VectorX<T>& DeformableDriver<T>::EvalParticipatingFreeMotionVelocities(
    const Context<T>& context) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.participating_free_motion_velocities)
      .template Eval<VectorX<T>>(context);
}

template class DeformableDriver<double>;
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::Multiplexer);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
