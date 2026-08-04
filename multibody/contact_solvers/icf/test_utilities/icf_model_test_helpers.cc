#include "drake/multibody/contact_solvers/icf/test_utilities/icf_model_test_helpers.h"

#include <memory>
#include <utility>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using Eigen::VectorXd;

template <typename T>
void MakeUnconstrainedModel(IcfModel<T>* model, bool single_clique,
                            double time_step) {
  const int nv = 18;

  // Release the parameters in order to set them.
  std::unique_ptr<IcfParameters<T>> params = model->ReleaseParameters();
  DRAKE_THROW_UNLESS(params != nullptr);

  params->time_step = time_step;
  params->v0 = VectorX<T>::LinSpaced(nv, -1.0, 1.0);

  // Define a sparse mass matrix with three cliques.
  const Matrix6<T> A1 = 0.3 * Matrix6<T>::Identity();
  const Matrix6<T> A2 = 2.3 * Matrix6<T>::Identity();
  const Matrix6<T> A3 = 1.5 * Matrix6<T>::Identity();

  // Set the dense mass matrix.
  MatrixX<T>& M0 = params->M0;
  M0 = MatrixX<T>::Identity(nv, nv);
  M0.template block<6, 6>(0, 0) = A1;
  M0.template block<6, 6>(6, 6) = A2;
  M0.template block<6, 6>(12, 12) = A3;

  // Define joint damping D₀.
  params->D0 = VectorX<T>::Constant(nv, 0.1);

  // Set coriolis, centrifugal, and gravitational terms k₀.
  params->k0 = VectorX<T>::LinSpaced(nv, -1.0, 1.0);

  // Define the clique structure for the sparse linearized dynamics matrix A.
  std::vector<int>& clique_sizes = params->clique_sizes;
  if (single_clique) {
    clique_sizes = {nv};
  } else {
    clique_sizes = {6, 6, 6};
  }

  // We use non-identity Jacobians to stress-test the algebra.
  if (single_clique) {
    // Floating bodies don't make sense in a single-clique model.
    params->body_is_floating = {0, 0, 0, 0};
  } else {
    params->body_is_floating = {0, 0, 1, 0};
  }
  params->body_mass = {1.0e20, 0.3, 2.3, 1.5};  // First body is the world.
  const Matrix6<T> J_WB0 = VectorX<T>::LinSpaced(36, -1.0, 1.0).reshaped(6, 6);
  const Matrix6<T> J_WB1 = Matrix6<T>::Identity();  // For floating body.
  const Matrix6<T> J_WB2 = J_WB0.transpose();

  if (single_clique) {
    params->J_WB.Resize(4, 6, 18);
    params->J_WB[0].setZero();
    params->J_WB[0].template block<6, 6>(0, 0) = Matrix6<T>::Identity();
    typename EigenPool<Matrix6X<T>>::MatrixView J0 = params->J_WB[1];
    J0.setZero();
    J0.template block<6, 6>(0, 0) = J_WB0;
    typename EigenPool<Matrix6X<T>>::MatrixView J1 = params->J_WB[2];
    J1.setZero();
    J1.template block<6, 6>(0, 6) = J_WB1;
    typename EigenPool<Matrix6X<T>>::MatrixView J2 = params->J_WB[3];
    J2.setZero();
    J2.template block<6, 6>(0, 12) = J_WB2;
  } else {
    params->J_WB.Resize(4, 6, 6);
    params->J_WB[0] = Matrix6<T>::Identity();  // World.
    params->J_WB[1] = J_WB0;
    params->J_WB[2] = J_WB1;
    params->J_WB[3] = J_WB2;
  }

  // We have three bodies and the world. The world is body -1.
  if (single_clique) {
    // All bodies in the same clique (clique 0).
    params->body_to_clique = {-1, 0, 0, 0};
  } else {
    // Each body in its own clique.
    params->body_to_clique = {-1, 0, 1, 2};
  }

  // No joint locking.
  auto& reduction = params->reduction;
  reduction.unlocked_dofs = {0,  1,  2,  3,  4,  5,   // BR
                             6,  7,  8,  9,  10, 11,  //
                             12, 13, 14, 15, 16, 17};
  if (single_clique) {
    reduction.per_clique_unlocked_dofs.resize(1);
    reduction.per_clique_unlocked_dofs.at(0) = reduction.unlocked_dofs;
  } else {
    reduction.per_clique_unlocked_dofs.resize(3);
    reduction.per_clique_unlocked_dofs.at(0) = {0, 1, 2, 3, 4, 5};
    reduction.per_clique_unlocked_dofs.at(1) = {0, 1, 2, 3, 4, 5};
    reduction.per_clique_unlocked_dofs.at(2) = {0, 1, 2, 3, 4, 5};
  }

  model->ResetParameters(std::move(params));
}

template <typename T>
void MakeModelReducible(IcfModel<T>* model,
                        const std::vector<int>& dofs_to_remove) {
  DRAKE_DEMAND(model != nullptr);
  const int nv = model->num_velocities();
  const int nc = model->num_cliques();
  DRAKE_DEMAND(ssize(dofs_to_remove) <= nv);
  DRAKE_DEMAND(std::ranges::is_sorted(dofs_to_remove));
  DRAKE_DEMAND(std::ranges::adjacent_find(dofs_to_remove) ==
               dofs_to_remove.end());
  DRAKE_DEMAND(dofs_to_remove.empty() || std::ranges::max(dofs_to_remove) < nv);
  DRAKE_DEMAND(dofs_to_remove.empty() || std::ranges::min(dofs_to_remove) >= 0);

  std::vector<int> all_dofs(nv);
  std::iota(all_dofs.begin(), all_dofs.end(), 0);

  std::unique_ptr<IcfParameters<T>> params = model->ReleaseParameters();
  typename IcfParameters<T>::ReductionParameters* r = &params->reduction;

  r->unlocked_dofs.clear();
  std::ranges::set_difference(all_dofs, dofs_to_remove,
                              std::back_inserter(r->unlocked_dofs));

  r->per_clique_unlocked_dofs.resize(nc);
  for (int c = 0; c < nc; ++c) {
    r->per_clique_unlocked_dofs.at(c).clear();
  }
  int remove_cursor{0};
  int clique_cursor{0};
  for (int v = 0; v < nv; ++v) {
    if (v >= model->clique_starts().at(clique_cursor + 1)) {
      ++clique_cursor;
    }
    const int clique_v = v - model->clique_starts().at(clique_cursor);
    if ((remove_cursor >= ssize(dofs_to_remove)) ||
        (v < dofs_to_remove.at(remove_cursor))) {
      r->per_clique_unlocked_dofs.at(clique_cursor).push_back(clique_v);
    } else if (v == dofs_to_remove.at(remove_cursor)) {
      ++remove_cursor;
    } else {
      DRAKE_UNREACHABLE();
    }
  }

  // Check for partially-locked free/floating bodies; that is not supported.
  for (int b = 0; b < model->num_bodies(); ++b) {
    if (params->body_is_floating.at(b)) {
      const int clique = params->body_to_clique.at(b);
      const int clique_size = ssize(r->per_clique_unlocked_dofs.at(clique));
      DRAKE_DEMAND(clique_size == 6 || clique_size == 0);
    }
  }

  model->ResetParameters(std::move(params));
}

template <typename T>
void AddCouplerConstraint(IcfModel<T>* model) {
  DRAKE_DEMAND(model != nullptr);
  const bool single_clique = model->num_cliques() == 1;

  CouplerConstraintsPool<T>& couplers = model->coupler_constraints_pool();
  couplers.Resize(1 /* one constraint */);

  const int nv = model->num_velocities();
  const VectorX<T> q0 = VectorX<T>::LinSpaced(nv, -1.0, 1.0);

  if (!single_clique) {
    // This is a multi-clique model, so we'll put a coupler on clique 1.
    const VectorX<T>& q0_clique = model->clique_segment(1, q0);
    const double rho = 2.5;
    const double offset = 0.1;
    couplers.Set(0 /* constraint index */, 1 /* clique */, 1 /* i */, 3 /* j */,
                 q0_clique(1), q0_clique(3), rho, offset);
  } else {
    // This is a single-clique model, so we'll put a coupler on clique 0.
    // However, we'll choose indices that would have belonged to clique 1 in the
    // multi-clique version.
    const VectorX<T>& q0_clique = model->clique_segment(0, q0);
    const double rho = 2.5;
    const double offset = 0.1;
    couplers.Set(0 /* constraint index */, 0 /* clique */, 7 /* i */, 9 /* j */,
                 q0_clique(7), q0_clique(9), rho, offset);
  }
}

template <typename T>
std::vector<VectorX<T>> AddGainConstraints(IcfModel<T>* model) {
  DRAKE_DEMAND(model != nullptr);
  const bool single_clique = model->num_cliques() == 1;
  GainConstraintsPool<T>& gain_constraints = model->gain_constraints_pool();

  // Define the constraint coefficients for two gain constraints. We'll define
  // the coefficients such that some constraints have values below, above, and
  // within the limits.
  const VectorX<T> K0 = VectorX<T>::Constant(6, 1.1);
  const VectorX<T> u0 = VectorX<T>::Constant(6, -6.0);
  const VectorX<T> e0 = VectorX<T>::Constant(6, 0.9);

  const VectorX<T> K1(
      (VectorX<T>(6) << 2.3, 0.0, 0.0, 2.3, 0.0, 2.3).finished());
  const VectorX<T> u1(
      (VectorX<T>(6) << -0.5, -13.5, -5.5, -0.5, 15.2, -0.5).finished());
  const VectorX<T> e1 = VectorX<T>::Constant(6, 11.1);

  if (!single_clique) {
    // We'll put the constraints on cliques 0 and 2.
    DRAKE_DEMAND(model->clique_size(0) == 6);
    DRAKE_DEMAND(model->clique_size(2) == 6);
    const std::vector<int> actuated_clique_sizes = {6, 6};

    gain_constraints.Resize(actuated_clique_sizes);
    gain_constraints.Set(0, 0, K0, u0, e0);
    gain_constraints.Set(1, 2, K1, u1, e1);

  } else {
    // We'll put the constraints on clique 0 only, since there is only one
    // clique. However, we'll define them such that the constraints are the same
    // as in the multi-clique version.
    DRAKE_DEMAND(model->clique_size(0) == 18);
    const std::vector<int> actuated_clique_sizes = {18};

    VectorX<T> K = VectorX<T>::Zero(18);
    VectorX<T> u = VectorX<T>::Zero(18);
    VectorX<T> e = VectorX<T>::Zero(18);
    K.template segment<6>(0) = K0;
    K.template segment<6>(12) = K1;
    u.template segment<6>(0) = u0;
    u.template segment<6>(12) = u1;
    e.template segment<6>(0) = e0;
    e.template segment<6>(12) = e1;

    gain_constraints.Resize(actuated_clique_sizes);
    gain_constraints.Set(0, 0, K, u, e);
  }

  return {K0, u0, e0, K1, u1, e1};
}

template <typename T>
void AddLimitConstraints(IcfModel<T>* model) {
  DRAKE_DEMAND(model != nullptr);
  const bool single_clique = model->num_cliques() == 1;
  LimitConstraintsPool<T>& limits = model->limit_constraints_pool();

  const int nv = model->num_velocities();
  const VectorX<T> q0 = VectorXd::LinSpaced(nv, -1.0, 1.0);

  if (!single_clique) {
    DRAKE_DEMAND(model->clique_size(0) == 6);
    DRAKE_DEMAND(model->clique_size(2) == 6);
    // Cliques 0 and 2 will have limits. We'll resize the constraint pool
    // accordingly before adding the limit constraints.
    std::vector<int> limited_clique_sizes = {model->clique_size(0),
                                             model->clique_size(2)};
    limits.Resize(limited_clique_sizes);

    // Limits on clique 0.
    limits.Set(0 /* constraint */, 0 /* clique */, 4 /* dof */, q0(2), -1.5,
               -0.5);
    limits.Set(0 /* constraint */, 0 /* clique */, 3 /* dof */, q0(3), -1.0,
               0.8);

    // Limits on clique 2.
    limits.Set(1 /* constraint */, 2 /* clique */, 0 /* dof */, q0(12), -0.9,
               1.5);
    limits.Set(1 /* constraint */, 2 /* clique */, 2 /* dof */, q0(14), -1.0,
               0.7);
    limits.Set(1 /* constraint */, 2 /* clique */, 5 /* dof */, q0(17), 0.5,
               1.5);
  } else {
    DRAKE_DEMAND(model->clique_size(0) == 18);
    // All limits will be on clique 0, but the same as in the multi-clique case.
    std::vector<int> limited_clique_sizes = {model->clique_size(0)};
    limits.Resize(limited_clique_sizes);
    limits.Set(0 /* constraint */, 0 /* clique */, 4 /* dof */, q0(2), -1.5,
               -0.5);
    limits.Set(0 /* constraint */, 0 /* clique */, 3 /* dof */, q0(3), -1.0,
               0.8);
    limits.Set(0 /* constraint */, 0 /* clique */, 12 /* dof */, q0(12), -0.9,
               1.5);
    limits.Set(0 /* constraint */, 0 /* clique */, 14 /* dof */, q0(14), -1.0,
               0.7);
    limits.Set(0 /* constraint */, 0 /* clique */, 17 /* dof */, q0(17), 0.5,
               1.5);
  }
}

template <typename T>
void AddPatchConstraints(IcfModel<T>* model) {
  // Add contact patches.
  const T dissipation = 50.0;
  const T stiffness = 1.0e6;
  const T friction = 0.5;

  PatchConstraintsPool<T>& patches = model->patch_constraints_pool();
  // Resize to hold three patches with one contact pair each.
  const std::vector<int> num_pairs_per_patch = {1, 1, 1};
  patches.Resize(num_pairs_per_patch);

  // First patch is between two non-anchored bodies, with body A floating.
  {
    const Vector3<T> p_AB_W(0.1, 0.0, 0.0);
    patches.SetPatch(0 /* patch index */, 2 /* body A */, 1 /* body B */,
                     dissipation, friction, friction, p_AB_W);

    const Vector3<T> nhat_AB_W(1.0, 0.0, 0.0);
    const Vector3<T> p_BC_W = -0.5 * p_AB_W;
    const T fn0 = 1.5;
    patches.SetPair(0 /* patch index */, 0 /* pair index */, p_BC_W, nhat_AB_W,
                    fn0, stiffness);
  }

  // Second patch is between two non-anchored bodies, with body B floating.
  {
    const Vector3<T> p_AB_W(-0.1, 0.0, 0.0);
    patches.SetPatch(2 /* patch index */, 1 /* body A */, 2 /* body B */,
                     dissipation, friction, friction, p_AB_W);

    const Vector3<T> nhat_AB_W(-1.0, 0.0, 0.0);
    const Vector3<T> p_BC_W = -0.5 * p_AB_W;
    const T fn0 = 1.5;
    patches.SetPair(2 /* patch index */, 0 /* pair index */, p_BC_W, nhat_AB_W,
                    fn0, stiffness);
  }

  // Third patch is between an anchored body (the world) and a dynamic body.
  {
    const Vector3<T> p_AB_W(0.0, 0.05, 0.0);
    patches.SetPatch(1 /* patch index */, 0 /* World */, 3 /* body B */,
                     dissipation, friction, friction, p_AB_W);

    const Vector3<T> nhat_AB_W(0.0, 1.0, 0.0);
    const Vector3<T> p_BC_W(0.0, -0.05, 0.0);
    const T fn0 = 1.5;
    patches.SetPair(1 /* patch index */, 0 /* pair index */, p_BC_W, nhat_AB_W,
                    fn0, stiffness);
  }
}

template <typename T>
void AddWeldConstraints(IcfModel<T>* model) {
  DRAKE_DEMAND(model != nullptr);

  WeldConstraintsPool<T>& welds = model->weld_constraints_pool();
  welds.Resize(2);

  // Weld 0: world (body 0, anchored) to body 1.
  {
    const Vector3<T> p_AP_W(0.1, 0.0, 0.0);
    const Vector3<T> p_BQ_W(0.0, 0.0, 0.0);
    const Vector3<T> p_PoQo_W(0.05, 0.0, 0.0);
    const Vector3<T> a_PQ_W(0.0, 0.0, 0.01);
    welds.Set(0, /*bodyA=*/0, /*bodyB=*/1, p_AP_W, p_BQ_W, p_PoQo_W, a_PQ_W);
  }

  // Weld 1: body 2 to body 3 (cross-clique in multi-clique case).
  {
    const Vector3<T> p_AP_W(0.1, 0.2, 0.3);
    const Vector3<T> p_BQ_W(0.0, 0.1, 0.0);
    const Vector3<T> p_PoQo_W(0.02, -0.01, 0.03);
    const Vector3<T> a_PQ_W(0.005, -0.003, 0.001);
    welds.Set(1, /*bodyA=*/2, /*bodyB=*/3, p_AP_W, p_BQ_W, p_PoQo_W, a_PQ_W);
  }
}

template <typename T>
void AddBallConstraints(IcfModel<T>* model) {
  DRAKE_DEMAND(model != nullptr);

  BallConstraintsPool<T>& ball_constraints = model->ball_constraints_pool();
  ball_constraints.Resize(2);

  // Ball 0: world (body 0, anchored) to body 1.
  {
    const Vector3<T> p_AP_W(0.1, 0.0, 0.0);
    const Vector3<T> p_BQ_W(0.0, 0.0, 0.0);
    const Vector3<T> p_PoQo_W(0.05, 0.0, 0.0);
    ball_constraints.Set(0, /*bodyA=*/0, /*bodyB=*/1, p_AP_W, p_BQ_W, p_PoQo_W);
  }

  // Ball 1: body 2 to body 3 (cross-clique in multi-clique case).
  {
    const Vector3<T> p_AP_W(0.1, 0.2, 0.3);
    const Vector3<T> p_BQ_W(0.0, 0.1, 0.0);
    const Vector3<T> p_PoQo_W(0.02, -0.01, 0.03);
    ball_constraints.Set(1, /*bodyA=*/2, /*bodyB=*/3, p_AP_W, p_BQ_W, p_PoQo_W);
  }
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&MakeUnconstrainedModel<T>, &MakeModelReducible<T>, &AddBallConstraints<T>,
     &AddCouplerConstraint<T>, &AddGainConstraints<T>, &AddLimitConstraints<T>,
     &AddPatchConstraints<T>, &AddWeldConstraints<T>));

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
