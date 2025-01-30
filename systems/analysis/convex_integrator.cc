#include "drake/systems/analysis/convex_integrator.h"

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
using multibody::internal::GetPointContactStiffness;
using multibody::internal::TreeIndex;

template <class T>
void ConvexIntegrator<T>::DoInitialize() {
  const int nq = plant().num_positions();
  const int nv = plant().num_velocities();

  // TODO(vincekurtz): in the future we might want some fancy caching instead of
  // the workspace, but for now we'll just try to allocate most things here.
  workspace_.q.resize(nq);
  workspace_.v_star.resize(nv);
  workspace_.A.assign(tree_topology().num_trees(), MatrixX<T>(nv, nv));

  workspace_.M.resize(nv, nv);
  workspace_.k.resize(nv);
  workspace_.a.resize(nv);
  workspace_.f_ext = std::make_unique<MultibodyForces<T>>(plant());
}

template <class T>
bool ConvexIntegrator<T>::DoStep(const T& h) {
  // Get the plant context. Note that we assume the only continuous state is the
  // plant's, and there are no controllers connected to it.
  Context<T>& context =
      plant().GetMyMutableContextFromRoot(this->get_mutable_context());

  // Get stuff from the workspace
  VectorX<T>& q = workspace_.q;
  VectorX<T>& v_star = workspace_.v_star;
  std::vector<MatrixX<T>>& A = workspace_.A;
  SapSolverResults<T>& sap_results = workspace_.sap_results;

  // Set up the SAP problem
  CalcFreeMotionVelocities(context, h, &v_star);
  CalcLinearDynamicsMatrix(context, h, &A);
  SapContactProblem<T> problem(h, A, v_star);
  AddContactConstraints(context, &problem);

  // Solve for v_{t+h} with convex optimization
  // TODO(vincekurtz): implement custom solve with Hessian re-use
  Eigen::VectorBlock<const VectorX<T>> v0 = plant().GetVelocities(context);
  SapSolver<T> sap;  // TODO(vincekurtz): set sap parameters
  SapSolverStatus status = sap.SolveWithGuess(problem, v0, &sap_results);

  DRAKE_DEMAND(status == SapSolverStatus::kSuccess);

  // Set q_{t+h} = q_t + h N(q_t) v_{t+h}
  plant().MapVelocityToQDot(context, h * sap_results.v, &q);
  q += plant().GetPositions(context);

  plant().SetPositions(&context, q);
  plant().SetVelocities(&context, sap_results.v);

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

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ConvexIntegrator);
