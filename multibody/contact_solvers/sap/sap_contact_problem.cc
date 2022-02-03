#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <iostream>
#include <map>

#include "drake/common/drake_copyable.h"
#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/partial_permutation.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapConstraint<T>::SapConstraint(int clique, const MatrixX<T>& J)
    : num_constrained_dofs_(J.rows()), clique0_(clique), J0_(J) {
  DRAKE_DEMAND(clique >= 0);
}

template <typename T>
SapConstraint<T>::SapConstraint(int clique0, int clique1, const MatrixX<T>& J0,
                                const MatrixX<T>& J1)
    : num_constrained_dofs_(J0.rows()),
      clique0_(clique0),
      num_velocities0_(J0.cols()),
      clique1_(clique1),
      num_velocities1_(J1.cols()),
      J0_(J0),
      J1_(J1) {
  DRAKE_DEMAND(clique0 >= 0);
  DRAKE_DEMAND(clique1 >= 0);
  DRAKE_DEMAND(clique0 != clique1);
  DRAKE_DEMAND(J0.rows() == J1.rows());
}

template <typename T>
int SapConstraint<T>::num_cliques() const {
  return clique1_ < 0 ? 1 : 2;
}

template <typename T>
int SapConstraint<T>::num_constrained_dofs() const {
  return num_constrained_dofs_;
}

template <typename T>
int SapConstraint<T>::clique0() const {
  return clique0_;
}

template <typename T>
int SapConstraint<T>::clique1() const {
  return clique1_;
}

template <typename T>
const MatrixX<T>& SapConstraint<T>::clique0_jacobian() const {
  return J0_;
}

template <typename T>
const MatrixX<T>& SapConstraint<T>::clique1_jacobian() const {
  return J1_;
}

template <typename T>
SapFrictionConeConstraint<T>::SapFrictionConeConstraint(const Parameters& p,
                                                        int clique,
                                                        const MatrixX<T>& J,
                                                        const T& phi0)
    : SapConstraint<T>(clique, J), parameters_(p), phi0_(phi0) {
  DRAKE_DEMAND(p.mu >= 0.0);
  DRAKE_DEMAND(this->clique0_jacobian().rows() == 3);
}

template <typename T>
SapFrictionConeConstraint<T>::SapFrictionConeConstraint(
    const Parameters& p, int clique0, int clique1, const MatrixX<T>& J0,
    const MatrixX<T>& J1, const T& phi0)
    : SapConstraint<T>(clique0, clique1, J0, J1), parameters_(p), phi0_(phi0) {
  DRAKE_DEMAND(p.mu >= 0.0);
  DRAKE_DEMAND(this->clique0_jacobian().rows() == 3);
  DRAKE_DEMAND(this->clique1_jacobian().rows() == 3);
}

template <typename T>
VectorX<T> SapFrictionConeConstraint<T>::CalcBiasTerm(const T& time_step,
                                                      const T&) const {
  const T& taud = parameters_.dissipation_time_scale;
  const T vn_hat = -phi0_ / (time_step + taud);
  return Vector3<T>(0, 0, vn_hat);
}

template <typename T>
VectorX<T> SapFrictionConeConstraint<T>::CalcDiagonalRegularization(
    const T& time_step, const T& wi) const {
  using std::max;

  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const double beta_factor =
      parameters_.beta * parameters_.beta / (4.0 * M_PI * M_PI);

  const T& k = parameters_.stiffness;
  const T& taud = parameters_.dissipation_time_scale;

  const T Rn =
      max(beta_factor * wi, 1.0 / (time_step * k * (time_step + taud)));
  const T Rt = parameters_.sigma * wi;
  return Vector3<T>(Rt, Rt, Rn);
}

template <typename T>
void SapFrictionConeConstraint<T>::Project(
    const Eigen::Ref<const VectorX<T>>& y,
    const Eigen::Ref<const VectorX<T>>& R, EigenPtr<VectorX<T>> gamma,
    MatrixX<T>* dPdy) const {
  // Computes the "soft norm" ‖x‖ₛ defined by ‖x‖ₛ² = ‖x‖² + ε², where ε =
  // soft_tolerance. Using the soft norm we define the tangent vector as t̂ =
  // γₜ/‖γₜ‖ₛ, which is well defined event for γₜ = 0. Also gradients are well
  // defined and follow the same equations presented in [Castro et al., 2021]
  // where regular norms are simply replaced by soft norms.
  auto soft_norm =
      [eps = soft_tolerance_](const Eigen::Ref<const VectorX<T>>& x) -> T {
    using std::sqrt;
    return sqrt(x.squaredNorm() + eps * eps);
  };

  // We assume a regularization of the form R = (Rt, Rt, Rn).
  const T& Rt = R(0);
  const T& Rn = R(2);
  const T mu_hat = mu() * Rt / Rn;

  const auto yt = y.template head<2>();
  const T yr = soft_norm(yt);
  const T yn = y(2);
  const Vector2<T> that = yt / yr;

  if (dPdy != nullptr) {
    dPdy->resize(3, 3);  // no-op if already the proper size.
  }

  // Analytical projection of y onto the friction cone ℱ using the R norm.
  if (yr < mu() * yn) {
    // Region I, stiction.
    *gamma = y;
    if (dPdy) dPdy->setIdentity();
  } else if (-mu_hat * yr < yn && yn <= yr / mu()) {
    // Region II, sliding.

    // Common terms in both the projection and its gradient.
    const T mu_tilde2 = mu() * mu_hat;  // mu_tilde = mu * sqrt(Rt/Rn).
    const T factor = 1.0 / (1.0 + mu_tilde2);

    // Projection P(y).
    const T gn = (yn + mu_hat * yr) * factor;
    const Vector2<T> gt = mu() * gn * that;
    *gamma << gt, gn;
    // gamma->template head<2>() = gt;
    //(*gamma)(2) = gn;

    // Gradient.
    if (dPdy) {
      const Matrix2<T> P = that * that.transpose();
      const Matrix2<T> Pperp = Matrix2<T>::Identity() - P;

      // We split dPdy into separate blocks:
      //
      // dPdy = |dgt_dyt dgt_dyn|
      //        |dgn_dyt dgn_dyn|
      // where dgt_dyt ∈ ℝ²ˣ², dgt_dyn ∈ ℝ², dgn_dyt ∈ ℝ²ˣ¹ and dgn_dyn ∈ ℝ.
      const Matrix2<T> dgt_dyt = mu() * (gn / yr * Pperp + mu_hat * factor * P);
      const Vector2<T> dgt_dyn = mu() * factor * that;
      const RowVector2<T> dgn_dyt = mu_hat * factor * that.transpose();
      const T dgn_dyn = factor;

      dPdy->template topLeftCorner<2, 2>() = dgt_dyt;
      dPdy->template topRightCorner<2, 1>() = dgt_dyn;
      dPdy->template bottomLeftCorner<1, 2>() = dgn_dyt;
      (*dPdy)(2, 2) = dgn_dyn;
    }
  } else {  // yn <= -mu_hat * yr
    // Region III, no contact.
    gamma->setZero();
    if (dPdy) dPdy->setZero();
  }
}

template <typename T>
SapCouplerConstraint<T>::SapCouplerConstraint(const Parameters& p, int clique,
                                              const MatrixX<T>& J, const T& g0)
    : SapConstraint<T>(clique, J), parameters_(p), g0_(g0) {
  DRAKE_DEMAND(this->clique0_jacobian().rows() == 1);
}

template <typename T>
SapCouplerConstraint<T>::SapCouplerConstraint(
    const Parameters& p, int clique0, int clique1, const MatrixX<T>& J0,
    const MatrixX<T>& J1, const T& g0)
    : SapConstraint<T>(clique0, clique1, J0, J1), parameters_(p), g0_(g0) {
  DRAKE_DEMAND(this->clique0_jacobian().rows() == 1);
  DRAKE_DEMAND(this->clique1_jacobian().rows() == 1);
}

template <typename T>
VectorX<T> SapCouplerConstraint<T>::CalcBiasTerm(const T& time_step,
                                                 const T&) const {
  const T& taud = parameters_.dissipation_time_scale;
  const T v_hat = -g0_ / (time_step + taud);
  return Vector1<T>(v_hat);
}

template <typename T>
VectorX<T> SapCouplerConstraint<T>::CalcDiagonalRegularization(
    const T& time_step, const T& wi) const {
  using std::max;

  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const double beta_factor =
      parameters_.beta * parameters_.beta / (4.0 * M_PI * M_PI);

  const T& k = parameters_.stiffness;
  const T& taud = parameters_.dissipation_time_scale;

  const T R = max(beta_factor * wi, 1.0 / (time_step * k * (time_step + taud)));
  return Vector1<T>(R);
}

template <typename T>
void SapCouplerConstraint<T>::Project(
    const Eigen::Ref<const VectorX<T>>& y,
    const Eigen::Ref<const VectorX<T>>&, EigenPtr<VectorX<T>> gamma,
    MatrixX<T>* dPdy) const {
  // For this constraint the projection operator is the identity operator.      
  *gamma = y;
  if (dPdy != nullptr) {
    dPdy->resize(1, 1);  // no-op if already the proper size.
    dPdy->setIdentity();
  }
}

template <typename T>
SapContactProblem<T>::SapContactProblem(const T& time_step,
                                        std::vector<MatrixX<T>>&& A,
                                        VectorX<T>&& v_star)
    : time_step_(time_step), A_(std::move(A)), v_star_(std::move(v_star)) {
  nv_ = 0;
  for (const auto& Ac : A_) {
    DRAKE_DEMAND(Ac.rows() == Ac.cols());
    nv_ += Ac.rows();
  }
  DRAKE_DEMAND(v_star_.size() == nv_);
}

template <typename T>
SapContactProblem<T>::SapContactProblem(
    std::vector<MatrixX<T>>&& A, VectorX<T>&& v_star,
    std::vector<std::unique_ptr<SapConstraint<T>>>&& constraints)
    : A_(std::move(A)),
      v_star_(std::move(v_star)),
      constraints_(std::move(constraints)) {
  nv_ = 0;
  for (const auto& Ac : A_) {
    DRAKE_DEMAND(Ac.rows() == Ac.cols());
    nv_ += Ac.rows();
  }
  DRAKE_DEMAND(v_star_.size() == nv_);
}

template <typename T>
void SapContactProblem<T>::AddConstraint(std::unique_ptr<SapConstraint<T>> c) {
  DRAKE_DEMAND(c->clique0() < num_cliques());
  DRAKE_DEMAND(c->clique1() < num_cliques());
  DRAKE_DEMAND(c->clique0_jacobian().cols() == num_velocities(c->clique0()));
  if (c->clique1() >= 0) {
    DRAKE_DEMAND(c->clique1_jacobian().cols() == num_velocities(c->clique1()));
  }
  num_constrained_dofs_ += c->num_constrained_dofs();
  constraints_.push_back(std::move(c));  
}

template <typename T>
int SapContactProblem<T>::num_cliques() const {
  return A_.size();
}

template <typename T>
int SapContactProblem<T>::num_constraints() const {
  return constraints_.size();
}

template <typename T>
int SapContactProblem<T>::num_velocities() const {
  return nv_;
}

template <typename T>
int SapContactProblem<T>::num_velocities(int clique) const {
  DRAKE_DEMAND(0 <= clique && clique < num_cliques());
  return A_[clique].rows();
}

template <typename T>
const SapConstraint<T>& SapContactProblem<T>::get_constraint(int k) const {
  DRAKE_DEMAND(0 <= k && k < num_constraints());
  return *constraints_[k];
}

template <typename T>
const std::vector<MatrixX<T>>& SapContactProblem<T>::dynamics_matrix() const {
  return A_;
}

template <typename T>
ContactProblemGraph SapContactProblem<T>::MakeGraph() const {
  // We sort the constraint groups by clique pair lexicographically. This allow
  // us to provide an invariant when checking the correctness of the code. As an
  // additional invariant, this will us allow later on the enumerate
  // participating cliques in the order they were provided.
  // TODO(amcastro-tri): while these invariants are nice to have an
  // unordered_map might be more efficient. Try this if profiling reveals a
  // bottleneck here.
  std::map<SortedPair<int>, std::vector<int>> constraint_groups;
  for (size_t k = 0; k < constraints_.size(); ++k) {
    const auto& c = constraints_[k];
    const int c0 = c->clique0();  // N.B. we know that clique0() > 0 always.
    // The create a "loop" to signify a constraint within the same clique.
    const int c1 = c->clique1() >= 0 ? c->clique1() : c->clique0();
    const auto cliques_pair = drake::MakeSortedPair(c0, c1);
    constraint_groups[cliques_pair].push_back(k);
  }

  const int num_edges = constraint_groups.size();
  ContactProblemGraph graph(num_cliques(), num_edges);
  for (auto& e : constraint_groups) {
    graph.AddConstraintGroup(ContactProblemGraph::ConstraintGroup(e.first, std::move(e.second)));
  }

  return graph;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapFrictionConeConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapCouplerConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapContactProblem)
