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
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

#if 0
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
#endif 

template <typename T>
class SapConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapConstraint);

  // Default constructor allow us to place SapConstraint objects in STL
  // containers.
  SapConstraint() = default;

  virtual ~SapConstraint() = default;

  // TODO: do I need this constraint?
  explicit SapConstraint(int num_constrained_dofs);

  // Costructor for a constraint among dofs within a single `clique`.
  SapConstraint(int clique, const MatrixX<T>& J);

  // Constructor for a constraint among dofs in two cliques `clique0` and
  // `clique1`.
  SapConstraint(int clique0, int clique1, const MatrixX<T>& J0,
                const MatrixX<T>& J1);

  // Number of dofs constrained by this constraint.
  int num_constrained_dofs() const;

  // Number of participating cliques. One (1) or two (2).
  int num_cliques() const;

  // Index of the first (and maybe only) participating clique. Always a
  // non-negative number.
  int clique0() const;

  // Index of the second participating clique. It is negative is there is only a
  // single clique that participates.
  int clique1() const;

  // Computes the projection γ = P(y) onto the convex set specific to a
  // constraint in the norm defined by the diagonal positive matrix R, i.e. the
  // norm ‖x‖ = xᵀ⋅R⋅x.
  // @param[in] y Impulse to be projected, of size num_constrained_dofs().
  // @param[in] R Specifies the diagonal components of matrix R, of size
  // num_constrained_dofs().
  // @param[out] gamma On output, the projected impulse y. On input it must be
  // of size num_constrained_dofs().
  // @param[out] Not used if nullptr. Otherwise it must be a squared matrix of
  // size num_constrained_dofs() to store the Jacobian dP/dy on output.
  virtual void Project(const Eigen::Ref<const VectorX<T>>& y,
                       const Eigen::Ref<const VectorX<T>>& R,
                       EigenPtr<VectorX<T>> gamma,
                       MatrixX<T>* dPdy = nullptr) const = 0;

  const MatrixX<T>& clique0_jacobian() const;
  const MatrixX<T>& clique1_jacobian() const;

  // Computes the bias term v̂ used to compute the constraint impulses before
  // projection as y = −R⁻¹⋅(vc − v̂).
  // @param[in] time_step The time step used in the temporal discretization.
  // @param[in] wi Constraint Delassus approximation (inverse of mass). Specific
  // constraints can use this information to estimate stabilization terms in the
  // "near-rigid" regime.
  virtual VectorX<T> CalcBiasTerm(const T& time_step, const T& wi) const = 0;

  // Computes the regularization R used to compute the constraint impulses
  // before projection as y = −R⁻¹⋅(vc − v̂).
  // @param[in] time_step The time step used in the temporal discretization.
  // @param[in] wi Constraint Delassus approximation (inverse of mass). Specific
  // constraints can use this information to estimate stabilization terms in the
  // "near-rigid" regime.
  virtual VectorX<T> CalcDiagonalRegularization(const T& time_step,
                                                const T& wi) const = 0;

 private:
  int num_constrained_dofs_{0};
  // For now we limit ourselves to constraints between two cliques only.
  int clique0_{-1};
  int num_velocities0_{0};
  int clique1_{-1};  
  int num_velocities1_{0};
  MatrixX<T> J0_;
  MatrixX<T> J1_;
};

template <typename T>
class SapFrictionConeConstraint final : public SapConstraint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapFrictionConeConstraint);

  // TODO: Simplify, get rid of struct Parameters.
  struct Parameters {
    T mu{0.0};
    T stiffness{0.0};
    T dissipation_time_scale{0.0};
    T phi0{0.0};
  };

  SapFrictionConeConstraint(const Parameters& p)
      : SapConstraint<T>(3), parameters_(p) {}

  // @throws if the number of rows in J is different from three.
  SapFrictionConeConstraint(int clique, const MatrixX<T>& J,
                            const Parameters& p);

  // @throws if the number of rows in J0 and J1 is different from three.
  SapFrictionConeConstraint(int clique0, int clique1, const MatrixX<T>& J0,
                            const MatrixX<T>& J1, const Parameters& p);

  const T& mu() const { return parameters_.mu; }                            

  void Project(const Eigen::Ref<const VectorX<T>>& y,
               const Eigen::Ref<const VectorX<T>>& R,
               EigenPtr<VectorX<T>> gamma,
               MatrixX<T>* dPdy = nullptr) const final;

  VectorX<T> CalcBiasTerm(const T& time_step, const T& wi) const final;
  VectorX<T> CalcDiagonalRegularization(const T& time_step,
                                        const T& wi) const final;

 private:
  Parameters parameters_;
  // Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the contact frequency ωₙ
  // is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. w
  // corresponds to a diagonal approximation of the Delassuss operator for each
  // contact. See [Castro et al., 2021. §IX.A] for details.
  double beta_{1.0};
  // Dimensionless parameterization of the regularization of friction. An
  // approximation for the bound on the slip velocity is vₛ ≈ σ⋅δt⋅g.
  double sigma_{1.0e-3};
  double soft_tolerance_{1.0e-7};
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
