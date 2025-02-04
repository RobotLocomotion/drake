#include "drake/systems/analysis/convex_integrator.h"

#include <iostream>

#include "drake/multibody/contact_solvers/sap/sap_hunt_crossley_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/plant/contact_properties.h"
#include "drake/multibody/plant/geometry_contact_data.h"

namespace drake {
namespace systems {

using drake::geometry::PenetrationAsPointPair;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapSolverStatus;
using multibody::DiscreteContactApproximation;
using multibody::Frame;
using multibody::RigidBody;
using multibody::contact_solvers::internal::MakeContactConfiguration;
using multibody::contact_solvers::internal::MatrixBlock;
using multibody::contact_solvers::internal::SapConstraintJacobian;
using multibody::contact_solvers::internal::SapHuntCrossleyApproximation;
using multibody::contact_solvers::internal::SapHuntCrossleyConstraint;
using multibody::internal::GetCombinedDissipationTimeConstant;
using multibody::internal::GetCombinedDynamicCoulombFriction;
using multibody::internal::GetCombinedHuntCrossleyDissipation;
using multibody::internal::GetCombinedPointContactStiffness;
using multibody::internal::GetHydroelasticModulus;
using multibody::internal::GetPointContactStiffness;
using multibody::internal::TreeIndex;

template <class T>
void ConvexIntegrator<T>::DoInitialize() {
  using std::isnan;

  const int nq = plant().num_positions();
  const int nv = plant().num_velocities();

  // TODO(vincekurtz): in the future we might want some fancy caching instead of
  // the workspace, but for now we'll just try to allocate most things here.
  workspace_.q.resize(nq);
  workspace_.q_h.resize(nq);
  workspace_.v_star.resize(nv);
  workspace_.A.assign(tree_topology().num_trees(), MatrixX<T>(nv, nv));
  workspace_.M.resize(nv, nv);
  workspace_.k.resize(nv);
  workspace_.a.resize(nv);
  workspace_.f_ext = std::make_unique<MultibodyForces<T>>(plant());
  workspace_.err.resize(nq + nv);
  
  // Set an artificial step size target, if not set already.
  if (isnan(this->get_initial_step_size_target())) {
    // Verify that maximum step size has been set.
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error("Neither initial step size target nor maximum "
                                 "step size has been set!");

    this->request_initial_step_size_target(
        this->get_maximum_step_size());
  }

}

template <class T>
bool ConvexIntegrator<T>::DoStep(const T& h) {
  // Get the plant context. Note that we assume the only continuous state is the
  // plant's, and there are no controllers connected to it.
  Context<T>& context =
      plant().GetMyMutableContextFromRoot(this->get_mutable_context());

  std::cout << "time: " << context.get_time();
  std::cout << " dt: " << h << std::endl;

  // Get stuff from the workspace
  VectorX<T>& q = workspace_.q;
  VectorX<T>& q_h = workspace_.q_h;
  VectorX<T>& v_star = workspace_.v_star;
  std::vector<MatrixX<T>>& A = workspace_.A;
  SapSolverResults<T>& sap_results = workspace_.sap_results;
  SapSolverResults<T>& sap_results_h = workspace_.sap_results_h;
  VectorX<T>& err = workspace_.err;

  // Set up the SAP problem
  CalcFreeMotionVelocities(context, h, &v_star);
  CalcLinearDynamicsMatrix(context, h, &A);
  SapContactProblem<T> problem(h, A, v_star);
  problem.set_num_objects(plant().num_bodies());
  AddContactConstraints(context, &problem);

  // Solve for v_{t+h} with convex optimization
  // TODO(vincekurtz): implement custom solve with Hessian re-use
  Eigen::VectorBlock<const VectorX<T>> v0 = plant().GetVelocities(context);
  SapSolver<T> sap;  // TODO(vincekurtz): set sap parameters
  SapSolverStatus status = sap.SolveWithGuess(problem, v0, &sap_results);
  DRAKE_DEMAND(status == SapSolverStatus::kSuccess);

  // Solve for v_{t+h} with two half-sized steps for error estimation
  // TODO(vincekurtz): reuse more info from the full-sized step
  CalcFreeMotionVelocities(context, 0.5 * h, &v_star);
  CalcLinearDynamicsMatrix(context, 0.5 * h, &A);
  SapContactProblem<T> problem_h1(0.5 * h, A, v_star);
  problem_h1.set_num_objects(plant().num_bodies());
  AddContactConstraints(context, &problem_h1);
  SapSolverStatus status_h1 = sap.SolveWithGuess(problem_h1, v0, &sap_results_h);
  DRAKE_DEMAND(status_h1 == SapSolverStatus::kSuccess);

  plant().MapVelocityToQDot(context, 0.5 * h * sap_results_h.v, &q_h);
  q_h += plant().GetPositions(context);
  plant().SetPositions(&context, q_h);
  plant().SetVelocities(&context, sap_results_h.v);

  CalcFreeMotionVelocities(context, 0.5 * h, &v_star);  // new state in context
  CalcLinearDynamicsMatrix(context, 0.5 * h, &A);
  SapContactProblem<T> problem_h2(0.5 * h, A, v_star);
  problem_h2.set_num_objects(plant().num_bodies());
  AddContactConstraints(context, &problem_h2);
  SapSolverStatus status_h2 = sap.SolveWithGuess(problem_h2, sap_results.v, &sap_results_h);
  DRAKE_DEMAND(status_h2 == SapSolverStatus::kSuccess);

  plant().MapVelocityToQDot(context, 0.5 * h * sap_results_h.v, &q_h);
  q_h += plant().GetPositions(context);
  plant().SetPositions(&context, q_h);
  plant().SetVelocities(&context, sap_results_h.v);

  // Update the error estimate
  err.head(plant().num_positions()) = q_h - q;
  err.tail(plant().num_velocities()) = sap_results_h.v - sap_results.v;
  this->get_mutable_error_estimate()->get_mutable_vector().SetFromVector(err);

  return true;  // step was successful
}

template <class T>
void ConvexIntegrator<T>::CalcFreeMotionVelocities(const Context<T>& context,
                                                   const T& h,
                                                   VectorX<T>* v_star) {
  DRAKE_DEMAND(v_star != nullptr);

  VectorX<T>& k = workspace_.k;
  VectorX<T>& a = workspace_.a;
  MatrixX<T>& M = workspace_.M;
  MultibodyForces<T>& f_ext = *workspace_.f_ext;
  Eigen::VectorBlock<const VectorX<T>> v0 = plant().GetVelocities(context);

  a.setZero();
  plant().CalcForceElementsContribution(context, &f_ext);
  k = plant().CalcInverseDynamics(context, a, f_ext);
  plant().CalcMassMatrix(context, &M);

  *v_star = v0 + h * M.ldlt().solve(-k);
}

template <class T>
void ConvexIntegrator<T>::CalcLinearDynamicsMatrix(const Context<T>& context,
                                                   const T& h,
                                                   std::vector<MatrixX<T>>* A) {
  DRAKE_DEMAND(A != nullptr);

  MatrixX<T>& M = workspace_.M;
  A->resize(tree_topology().num_trees());

  // TODO(vincekurtz): we could eventually cache M to avoid re-computing here
  // and in CalcFreeMotionVelocities.
  plant().CalcMassMatrix(context, &M);
  M.diagonal() += h * plant().EvalJointDampingCache(context);

  // Construct the (sparse) linear dynamics matrix A = M + h D
  for (TreeIndex t(0); t < tree_topology().num_trees(); ++t) {
    const int tree_start_in_v = tree_topology().tree_velocities_start_in_v(t);
    const int tree_nv = tree_topology().num_tree_velocities(t);
    (*A)[t] = M.block(tree_start_in_v, tree_start_in_v, tree_nv, tree_nv);
  }
}

template <class T>
void ConvexIntegrator<T>::AddContactConstraints(const Context<T>& context,
                                                SapContactProblem<T>* problem) {
  // N.B. this is essentially copy-pasted from SapDriver, with some
  // simplification to focus only on kLagged
  DRAKE_DEMAND(problem != nullptr);
  DRAKE_DEMAND(plant().get_discrete_contact_approximation() ==
               DiscreteContactApproximation::kLagged);
  constexpr double sigma = 1.0e-3;

  DiscreteContactData<DiscreteContactPair<T>>& contact_pairs =
      workspace_.contact_data;
  CalcContactPairs(context, &contact_pairs);
  const int num_contacts = contact_pairs.size();

  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& pair = contact_pairs[icontact];

    const T stiffness = pair.stiffness;
    const T damping = pair.damping;
    const T friction = pair.friction_coefficient;
    const auto& jacobian_blocks = pair.jacobian;

    auto make_hunt_crossley_parameters = [&]() {
      const double vs = plant().stiction_tolerance();
      SapHuntCrossleyApproximation model =
          SapHuntCrossleyApproximation::kLagged;
      return typename SapHuntCrossleyConstraint<T>::Parameters{
          model, friction, stiffness, damping, vs, sigma};
    };

    if (jacobian_blocks.size() == 1) {
      SapConstraintJacobian<T> J(jacobian_blocks[0].tree,
                                 std::move(jacobian_blocks[0].J));
      problem->AddConstraint(std::make_unique<SapHuntCrossleyConstraint<T>>(
          MakeContactConfiguration(pair), std::move(J),
          make_hunt_crossley_parameters()));
    } else {
      SapConstraintJacobian<T> J(
          jacobian_blocks[0].tree, std::move(jacobian_blocks[0].J),
          jacobian_blocks[1].tree, std::move(jacobian_blocks[1].J));
      problem->AddConstraint(std::make_unique<SapHuntCrossleyConstraint<T>>(
          MakeContactConfiguration(pair), std::move(J),
          make_hunt_crossley_parameters()));
    }
  }
}

template <class T>
void ConvexIntegrator<T>::CalcContactPairs(
    const Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* result) const {
  // N.B. this is essentially copy-pasted from
  // DiscreteUpdateManater::CalcDiscreteContactPairs.
  plant().ValidateContext(context);
  DRAKE_DEMAND(result != nullptr);
  result->Clear();
  AppendDiscreteContactPairsForPointContact(context, result);
  if constexpr (std::is_same_v<T, symbolic::Expression>) {
    throw std::logic_error("This method doesn't support T = Expression.");
  } else {
    AppendDiscreteContactPairsForHydroelasticContact(context, result);
  }
}

template <class T>
void ConvexIntegrator<T>::AppendDiscreteContactPairsForPointContact(
    const Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* contact_pairs) const {
  // N.B. this is essentially copy-pasted from DiscreteUpdateManager

  const std::vector<PenetrationAsPointPair<T>>& point_pairs =
      plant().EvalGeometryContactData(context).get().point_pairs;

  const int num_point_contacts = point_pairs.size();
  if (num_point_contacts == 0) {
    return;
  }

  contact_pairs->Reserve(num_point_contacts, 0, 0);
  const geometry::SceneGraphInspector<T>& inspector =
      plant().EvalSceneGraphInspector(context);
  const MultibodyTreeTopology& topology = internal_tree().get_topology();
  const Eigen::VectorBlock<const VectorX<T>> v = plant().GetVelocities(context);
  const Frame<T>& frame_W = plant().world_frame();

  // Scratch workspace variables.
  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAc_W(3, nv);
  Matrix3X<T> Jv_WBc_W(3, nv);
  Matrix3X<T> Jv_AcBc_W(3, nv);

  // Fill in the point contact pairs.
  for (int point_pair_index = 0; point_pair_index < num_point_contacts;
       ++point_pair_index) {
    const PenetrationAsPointPair<T>& pair = point_pairs[point_pair_index];
    const BodyIndex body_A_index = FindBodyByGeometryId(pair.id_A);
    const RigidBody<T>& body_A = plant().get_body(body_A_index);
    const BodyIndex body_B_index = FindBodyByGeometryId(pair.id_B);
    const RigidBody<T>& body_B = plant().get_body(body_B_index);

    const TreeIndex treeA_index = topology.body_to_tree_index(body_A_index);
    const TreeIndex treeB_index = topology.body_to_tree_index(body_B_index);
    const bool treeA_has_dofs = topology.tree_has_dofs(treeA_index);
    const bool treeB_has_dofs = topology.tree_has_dofs(treeB_index);

    const T kA = GetPointContactStiffness(
        pair.id_A, default_contact_stiffness(), inspector);
    const T kB = GetPointContactStiffness(
        pair.id_B, default_contact_stiffness(), inspector);

    // We compute the position of the point contact based on Hertz's theory
    // for contact between two elastic bodies.
    const T denom = kA + kB;
    const T wA = (denom == 0 ? 0.5 : kA / denom);
    const T wB = (denom == 0 ? 0.5 : kB / denom);
    const Vector3<T> p_WC = wA * pair.p_WCa + wB * pair.p_WCb;

    // Since v_AcBc_W = v_WBc - v_WAc the relative velocity Jacobian will
    // be:
    //   J_AcBc_W = Jv_WBc_W - Jv_WAc_W.
    // That is the relative velocity at C is v_AcBc_W = J_AcBc_W * v.
    internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_A.body_frame(), frame_W, p_WC,
        frame_W, frame_W, &Jv_WAc_W);
    internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_B.body_frame(), frame_W, p_WC,
        frame_W, frame_W, &Jv_WBc_W);
    Jv_AcBc_W = Jv_WBc_W - Jv_WAc_W;

    // Define a contact frame C at the contact point such that the z-axis Cz
    // equals nhat_W. The tangent vectors are arbitrary, with the only
    // requirement being that they form a valid right handed basis with
    // nhat_W.
    const Vector3<T> nhat_AB_W = -pair.nhat_BA_W;
    math::RotationMatrix<T> R_WC =
        math::RotationMatrix<T>::MakeFromOneVector(nhat_AB_W, 2);

    // Contact velocity stored in the current context (previous time step).
    const Vector3<T> v_AcBc_W = Jv_AcBc_W * v;
    const Vector3<T> v_AcBc_C = R_WC.transpose() * v_AcBc_W;
    const T vn0 = v_AcBc_C(2);

    // We have at most two blocks per contact.
    std::vector<typename DiscreteContactPair<T>::JacobianTreeBlock>
        jacobian_blocks;
    jacobian_blocks.reserve(2);

    // Tree A contribution to contact Jacobian Jv_W_AcBc_C.
    if (treeA_has_dofs) {
      Matrix3X<T> J =
          R_WC.matrix().transpose() *
          Jv_AcBc_W.middleCols(
              tree_topology().tree_velocities_start_in_v(treeA_index),
              tree_topology().num_tree_velocities(treeA_index));
      jacobian_blocks.emplace_back(treeA_index, MatrixBlock<T>(std::move(J)));
    }

    // Tree B contribution to contact Jacobian Jv_W_AcBc_C.
    // This contribution must be added only if B is different from A.
    if ((treeB_has_dofs && !treeA_has_dofs) ||
        (treeB_has_dofs && treeB_index != treeA_index)) {
      Matrix3X<T> J =
          R_WC.matrix().transpose() *
          Jv_AcBc_W.middleCols(
              tree_topology().tree_velocities_start_in_v(treeB_index),
              tree_topology().num_tree_velocities(treeB_index));
      jacobian_blocks.emplace_back(treeB_index, MatrixBlock<T>(std::move(J)));
    }

    // Contact stiffness and damping
    const T k = GetCombinedPointContactStiffness(
        pair.id_A, pair.id_B, default_contact_stiffness(), inspector);
    // Hunt & Crossley dissipation. Ignored, for instance, by Sap. See
    // multibody::DiscreteContactApproximation for details about these contact
    // models.
    const T d = GetCombinedHuntCrossleyDissipation(
        pair.id_A, pair.id_B, kA, kB, default_contact_dissipation(), inspector);
    // Dissipation time scale. Ignored, for instance, by Similar and Lagged
    // models. See multibody::DiscreteContactApproximation for details about
    // these contact models.
    const double default_dissipation_time_constant = 0.1;
    const T tau = GetCombinedDissipationTimeConstant(
        pair.id_A, pair.id_B, default_dissipation_time_constant, body_A.name(),
        body_B.name(), inspector);
    const T mu =
        GetCombinedDynamicCoulombFriction(pair.id_A, pair.id_B, inspector);

    const T phi0 = -pair.depth;
    const T fn0 = k * pair.depth;

    // Contact point position relative to each body.
    const RigidTransform<T>& X_WA =
        plant().EvalBodyPoseInWorld(context, body_A);
    const Vector3<T>& p_WA = X_WA.translation();
    const Vector3<T> p_AC_W = p_WC - p_WA;
    const RigidTransform<T>& X_WB =
        plant().EvalBodyPoseInWorld(context, body_B);
    const Vector3<T>& p_WB = X_WB.translation();
    const Vector3<T> p_BC_W = p_WC - p_WB;

    DiscreteContactPair<T> contact_pair{.jacobian = std::move(jacobian_blocks),
                                        .id_A = pair.id_A,
                                        .object_A = body_A_index,
                                        .id_B = pair.id_B,
                                        .object_B = body_B_index,
                                        .R_WC = R_WC,
                                        .p_WC = p_WC,
                                        .p_ApC_W = p_AC_W,
                                        .p_BqC_W = p_BC_W,
                                        .nhat_BA_W = pair.nhat_BA_W,
                                        .phi0 = phi0,
                                        .vn0 = vn0,
                                        .fn0 = fn0,
                                        .stiffness = k,
                                        .damping = d,
                                        .dissipation_time_scale = tau,
                                        .friction_coefficient = mu,
                                        .surface_index{} /* no surface index */,
                                        .face_index = {} /* no face index */,
                                        .point_pair_index = point_pair_index};
    contact_pairs->AppendPointData(std::move(contact_pair));
  }
}

template <class T>
void ConvexIntegrator<T>::AppendDiscreteContactPairsForHydroelasticContact(
    const Context<T>& context,
    DiscreteContactData<DiscreteContactPair<T>>* contact_pairs) const
  requires scalar_predicate<T>::is_bool
{  // NOLINT(whitespace/braces)
  const std::vector<geometry::ContactSurface<T>>& surfaces =
      plant().EvalGeometryContactData(context).get().surfaces;
  // N.B. For discrete hydro we use a first order quadrature rule. As such,
  // the per-face quadrature point is the face's centroid and the weight is 1.
  // This is compatible with a mesh that is triangle or polygon. If we attempted
  // higher order quadrature, polygons would have to be decomposed into smaller
  // n-gons which can receive an appropriate set of quadrature points.
  int num_hydro_contacts = 0;
  for (const auto& s : surfaces) {
    // One quadrature point per face.
    num_hydro_contacts += s.num_faces();
  }
  if (num_hydro_contacts == 0) {
    return;
  }

  contact_pairs->Reserve(0, num_hydro_contacts, 0);
  const geometry::SceneGraphInspector<T>& inspector =
      plant().EvalSceneGraphInspector(context);
  const MultibodyTreeTopology& topology = internal_tree().get_topology();
  const Eigen::VectorBlock<const VectorX<T>> v = plant().GetVelocities(context);
  const Frame<T>& frame_W = plant().world_frame();

  // Scratch workspace variables.
  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAc_W(3, nv);
  Matrix3X<T> Jv_WBc_W(3, nv);
  Matrix3X<T> Jv_AcBc_W(3, nv);

  const int num_surfaces = surfaces.size();
  for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
    const auto& s = surfaces[surface_index];

    const bool M_is_compliant = s.HasGradE_M();
    const bool N_is_compliant = s.HasGradE_N();
    DRAKE_DEMAND(M_is_compliant || N_is_compliant);

    // We always call the body associated with geometry M, A, and the body
    // associated with geometry N, B.
    const BodyIndex body_A_index = FindBodyByGeometryId(s.id_M());
    const RigidBody<T>& body_A = plant().get_body(body_A_index);
    const BodyIndex body_B_index = FindBodyByGeometryId(s.id_N());
    const RigidBody<T>& body_B = plant().get_body(body_B_index);

    const TreeIndex& tree_A_index = topology.body_to_tree_index(body_A_index);
    const TreeIndex& tree_B_index = topology.body_to_tree_index(body_B_index);
    const bool treeA_has_dofs = topology.tree_has_dofs(tree_A_index);
    const bool treeB_has_dofs = topology.tree_has_dofs(tree_B_index);

    // TODO(amcastro-tri): Consider making the modulus required, instead of
    // a default infinite value.
    const T hydro_modulus_M = GetHydroelasticModulus(
        s.id_M(), std::numeric_limits<double>::infinity(), inspector);
    const T hydro_modulus_N = GetHydroelasticModulus(
        s.id_N(), std::numeric_limits<double>::infinity(), inspector);
    // Hunt & Crossley dissipation. Used by the Tamsi, Lagged, and Similar
    // contact models. Ignored by Sap. See
    // multibody::DiscreteContactApproximation for details about these contact
    // models.
    const T d = GetCombinedHuntCrossleyDissipation(
        s.id_M(), s.id_N(), hydro_modulus_M, hydro_modulus_N,
        default_contact_dissipation(), inspector);
    // Dissipation time scale. Used by Sap contact model. Ignored by Tamsi,
    // Lagged, and Similar contact model. See
    // multibody::DiscreteContactApproximation for details about these contact
    // models.
    const double default_dissipation_time_constant = 0.1;
    const T tau = GetCombinedDissipationTimeConstant(
        s.id_M(), s.id_N(), default_dissipation_time_constant, body_A.name(),
        body_B.name(), inspector);
    // Combine friction coefficients.
    const T mu =
        GetCombinedDynamicCoulombFriction(s.id_M(), s.id_N(), inspector);

    for (int face = 0; face < s.num_faces(); ++face) {
      const T& Ae = s.area(face);  // Face element area.

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
        // From ContactSurface's documentation: The normal of each face is
        // guaranteed to point "out of" N and "into" M. Recall that A is
        // associated with M, and B is associated with N.
        const Vector3<T>& nhat_BA_W = s.face_normal(face);

        // One dimensional pressure gradient (in Pa/m). Unlike [Masterjohn
        // 2022], for convenience we define both pressure gradients
        // to be positive in the direction "into" the bodies. Therefore,
        // we use the minus sign for gN.
        // [Masterjohn 2022] Velocity Level Approximation of Pressure
        // Field Contact Patches.
        const T gM = M_is_compliant
                         ? s.EvaluateGradE_M_W(face).dot(nhat_BA_W)
                         : T(std::numeric_limits<double>::infinity());
        const T gN = N_is_compliant
                         ? -s.EvaluateGradE_N_W(face).dot(nhat_BA_W)
                         : T(std::numeric_limits<double>::infinity());

        constexpr double kGradientEpsilon = 1.0e-14;
        if (gM < kGradientEpsilon || gN < kGradientEpsilon) {
          // Mathematically g = gN*gM/(gN+gM) and therefore g = 0 when
          // either gradient on one of the bodies is zero. A zero gradient
          // means there is no contact constraint, and therefore we
          // ignore it to avoid numerical problems in the discrete solver.
          continue;
        }

        // Effective hydroelastic pressure gradient g result of
        // compliant-compliant interaction, see [Masterjohn 2022].
        // The expression below is mathematically equivalent to g =
        // gN*gM/(gN+gM) but it has the advantage of also being valid if
        // one of the gradients is infinity.
        const T g = 1.0 / (1.0 / gM + 1.0 / gN);

        // Position of quadrature point C in the world frame (since mesh_W
        // is measured and expressed in W).
        const Vector3<T>& p_WC = s.centroid(face);

        // Since v_AcBc_W = v_WBc - v_WAc the relative velocity Jacobian
        // will be:
        //   J_AcBc_W = Jv_WBc_W - Jv_WAc_W.
        // That is the relative velocity at C is v_AcBc_W = J_AcBc_W * v.
        internal_tree().CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable::kV, body_A.body_frame(), frame_W,
            p_WC, frame_W, frame_W, &Jv_WAc_W);
        internal_tree().CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable::kV, body_B.body_frame(), frame_W,
            p_WC, frame_W, frame_W, &Jv_WBc_W);
        Jv_AcBc_W = Jv_WBc_W - Jv_WAc_W;

        // Define a contact frame C at the contact point such that the
        // z-axis Cz equals nhat_AB_W. The tangent vectors are arbitrary,
        // with the only requirement being that they form a valid right
        // handed basis with nhat_AB_W.
        const Vector3<T> nhat_AB_W = -nhat_BA_W;
        math::RotationMatrix<T> R_WC =
            math::RotationMatrix<T>::MakeFromOneVector(nhat_AB_W, 2);

        // Contact velocity stored in the current context (previous time
        // step).
        const Vector3<T> v_AcBc_W = Jv_AcBc_W * v;
        const Vector3<T> v_AcBc_C = R_WC.transpose() * v_AcBc_W;
        const T vn0 = v_AcBc_C(2);

        // We have at most two blocks per contact.
        std::vector<typename DiscreteContactPair<T>::JacobianTreeBlock>
            jacobian_blocks;
        jacobian_blocks.reserve(2);

        // Tree A contribution to contact Jacobian Jv_W_AcBc_C.
        if (treeA_has_dofs) {
          Matrix3X<T> J =
              R_WC.matrix().transpose() *
              Jv_AcBc_W.middleCols(
                  tree_topology().tree_velocities_start_in_v(tree_A_index),
                  tree_topology().num_tree_velocities(tree_A_index));
          jacobian_blocks.emplace_back(tree_A_index,
                                       MatrixBlock<T>(std::move(J)));
        }

        // Tree B contribution to contact Jacobian Jv_W_AcBc_C.
        // This contribution must be added only if B is different from A.
        if ((treeB_has_dofs && !treeA_has_dofs) ||
            (treeB_has_dofs && tree_B_index != tree_A_index)) {
          Matrix3X<T> J =
              R_WC.matrix().transpose() *
              Jv_AcBc_W.middleCols(
                  tree_topology().tree_velocities_start_in_v(tree_B_index),
                  tree_topology().num_tree_velocities(tree_B_index));
          jacobian_blocks.emplace_back(tree_B_index,
                                       MatrixBlock<T>(std::move(J)));
        }

        // For a triangle, its centroid has the fixed barycentric
        // coordinates independent of the shape of the triangle. Using
        // barycentric coordinates to evaluate field value could be
        // faster than using Cartesian coordinates, especially if the
        // TriangleSurfaceMeshFieldLinear<> does not store gradients and
        // has to solve linear equations to convert Cartesian to
        // barycentric coordinates.
        const Vector3<T> tri_centroid_barycentric(1 / 3., 1 / 3., 1 / 3.);
        // Pressure at the quadrature point.
        const T p0 = s.is_triangle()
                         ? s.tri_e_MN().Evaluate(face, tri_centroid_barycentric)
                         : s.poly_e_MN().EvaluateCartesian(face, p_WC);

        // Force contribution by this quadrature point.
        const T fn0 = Ae * p0;

        // Effective compliance in the normal direction for the given
        // discrete patch, refer to [Masterjohn 2022] for details.
        // [Masterjohn 2022] Masterjohn J., Guoy D., Shepherd J. and
        // Castro A., 2022. Velocity Level Approximation of Pressure Field
        // Contact Patches. Available at https://arxiv.org/abs/2110.04157.
        const T k = Ae * g;

        // phi < 0 when in penetration.
        const T phi0 = -p0 / g;

        // Contact point position relative to each body.
        const RigidTransform<T>& X_WA =
            plant().EvalBodyPoseInWorld(context, body_A);
        const Vector3<T>& p_WA = X_WA.translation();
        const Vector3<T> p_AC_W = p_WC - p_WA;
        const RigidTransform<T>& X_WB =
            plant().EvalBodyPoseInWorld(context, body_B);
        const Vector3<T>& p_WB = X_WB.translation();
        const Vector3<T> p_BC_W = p_WC - p_WB;

        DiscreteContactPair<T> contact_pair{
            .jacobian = std::move(jacobian_blocks),
            .id_A = s.id_M(),
            .object_A = body_A_index,
            .id_B = s.id_N(),
            .object_B = body_B_index,
            .R_WC = R_WC,
            .p_WC = p_WC,
            .p_ApC_W = p_AC_W,
            .p_BqC_W = p_BC_W,
            .nhat_BA_W = nhat_BA_W,
            .phi0 = phi0,
            .vn0 = vn0,
            .fn0 = fn0,
            .stiffness = k,
            .damping = d,
            .dissipation_time_scale = tau,
            .friction_coefficient = mu,
            .surface_index = surface_index,
            .face_index = face,
            .point_pair_index = {} /* no point pair index */};
        contact_pairs->AppendHydroData(std::move(contact_pair));
      }
    }
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ConvexIntegrator);
