#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_problem_graph.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
class Projection {
 public:
  virtual ~Projection() = default;

  // The dimension the convex set C lives in.
  virtual int dimension() const = 0;

  virtual void Project(const Eigen::Ref<const VectorX<T>>& y,
                       const Eigen::Ref<const VectorX<T>>& R,
                       EigenPtr<VectorX<T>> gamma,
                       MatrixX<T>* dPdy = nullptr) const = 0;

 protected:
  Projection() = default;

  // This is *not* in the public section. However, this allows the children to
  // also use this macro, but precludes the possibility of slicing derived
  // projections.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Projection)
};

template <typename T>
class FrictionConeProjection final : public Projection<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrictionConeProjection);

  FrictionConeProjection(const T& mu) : mu_(mu) {}

  ~FrictionConeProjection() = default;

  int dimension() const final { return 3; }
  
  const T& mu() const { return mu_; }

  void Project(const Eigen::Ref<const VectorX<T>>& y,
               const Eigen::Ref<const VectorX<T>>& R,
               EigenPtr<VectorX<T>> gamma,
               MatrixX<T>* dPdy = nullptr) const final {
    // Computes the "soft norm" ‖x‖ₛ defined by ‖x‖ₛ² = ‖x‖² + ε², where ε =
    // soft_tolerance. Using the soft norm we define the tangent vector as t̂ =
    // γₜ/‖γₜ‖ₛ, which is well defined event for γₜ = 0. Also gradients are well
    // defined and follow the same equations presented in [Castro et al., 2021]
    // where regular norms are simply replaced by soft norms.
    auto soft_norm = [eps = soft_tolerance_](
                         const Eigen::Ref<const VectorX<T>>& x) -> T {
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
        const Matrix2<T> dgt_dyt =
            mu() * (gn / yr * Pperp + mu_hat * factor * P);
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

 private:
  T mu_{0.0};
  double soft_tolerance_{1.0e-7};
};

template <typename T>
class SapConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapConstraint);

  // Default constructor allow us to place SapConstraint objects in STL
  // containers.
  SapConstraint() = default;

  virtual ~SapConstraint() = default;

  SapConstraint(int clique, const MatrixX<T>& J);

  SapConstraint(int clique0, int clique1, const MatrixX<T>& J0,
                const MatrixX<T>& J1);

  int num_cliques() const;

  int num_constrained_dofs() const;

  int clique0() const;
  int clique1() const;

  const MatrixX<T>& clique0_jacobian() const;
  const MatrixX<T>& clique1_jacobian() const;

  virtual VectorX<T> Project(const Eigen::Ref<const VectorX<T>>& y,
                             std::optional<MatrixX<T>> dPdy) const = 0;

  virtual const Projection<T>& projection() const = 0;

  // VectorX<T> CalcConstraintBias(const T& wi);
  // VectorX<T> CalcRegularizationParameters(const T& wi);

 private:
  // For now we limit ourselves to constraints between two cliques only.
  int clique0_{-1};
  int clique1_{-1};
  MatrixX<T> J0_;
  MatrixX<T> J1_;
  int num_constrained_dofs_{0};
};

template <typename T>
class SapFrictionConeConstraint final : public SapConstraint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapFrictionConeConstraint);

  SapFrictionConeConstraint(int clique, const MatrixX<T>& J, const T& mu);

  SapFrictionConeConstraint(int clique0, int clique1, const MatrixX<T>& J0,
                            const MatrixX<T>& J1, const T& mu);

  const Projection<T>& projection() const final { return projection_; }

  VectorX<T> Project(const Eigen::Ref<const VectorX<T>>&,
                     std::optional<MatrixX<T>> dPdy) const final;

 private:
  FrictionConeProjection<T> projection_;
};

// SAP Problem defined by:
//   - A⋅(v−v*) = Jᵀ⋅γ
//   - Constraints.
template <typename T>
class SapContactProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapContactProblem);

  SapContactProblem(const T& time_step, std::vector<MatrixX<T>>&& A,
                    VectorX<T>&& v_star);

  SapContactProblem(
      std::vector<MatrixX<T>>&& A, VectorX<T>&& v_star,
      std::vector<std::unique_ptr<SapConstraint<T>>>&& constraints);

  void AddConstraint(std::unique_ptr<SapConstraint<T>> c);

  int num_cliques() const;

  int num_constraints() const;

  int num_constrained_dofs() const { return num_constrained_dofs_; }

  int num_velocities() const;

  int num_velocities(int clique) const;

  const SapConstraint<T>& get_constraint(int k) const;

  const T& time_step() const { return time_step_;  }

  // Returns the block diagonal dynamics matrix A.
  const std::vector<MatrixX<T>>& dynamics_matrix() const;

  const VectorX<T>& v_star() const { return v_star_;  }

  ContactProblemGraph MakeGraph() const;

 private:
  int nv_{0};
  T time_step_{0.0};
  std::vector<MatrixX<T>> A_;
  VectorX<T> v_star_;
  int num_constrained_dofs_{0};
  std::vector<std::unique_ptr<SapConstraint<T>>> constraints_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
