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

template <typename T>
class SapConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapConstraint);

  // Default constructor allow us to place SapConstraint objects in STL
  // containers.
  SapConstraint() = default;

  virtual ~SapConstraint() = default;

  // Costructor for a constraint among dofs within a single `clique`.
  // TODO: consider these constructors also take the function g. That way
  // essentially the constructors are provided with constraint function and
  // Jacobian values for the initial state. This seems nicely consistent.
  SapConstraint(int clique, const MatrixX<T>& J);

  // Constructor for a constraint among dofs in two cliques `clique0` and
  // `clique1`.
  // TODO: consider these constructors also take the function g. That way
  // essentially the constructors are provided with constraint function and
  // Jacobian values for the initial state. This seems nicely consistent.
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
  // TODO: Provide default implementation based on the value of g passed at
  // construction.
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
    // Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the contact frequency
    // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. w
    // corresponds to a diagonal approximation of the Delassuss operator for
    // each contact. See [Castro et al., 2021. §IX.A] for details.
    double beta{1.0};
    // Dimensionless parameterization of the regularization of friction. An
    // approximation for the bound on the slip velocity is vₛ ≈ σ⋅δt⋅g.
    double sigma{1.0e-3};
  };

  // @throws if the number of rows in J is different from three.
  SapFrictionConeConstraint(const Parameters& p, int clique,
                            const MatrixX<T>& J, const T& phi0);

  // @throws if the number of rows in J0 and J1 is different from three.
  SapFrictionConeConstraint(const Parameters& p, int clique0, int clique1,
                            const MatrixX<T>& J0, const MatrixX<T>& J1,
                            const T& phi0);

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
  T phi0_;
  double soft_tolerance_{1.0e-7};
};


// Imposes the constraint g = qᵢ − r⋅qⱼ that couples the i-th and j-th DOFs. r
// is the gear ratio, which can also be negative.
// The compliant constraint forces obeys the relationship: γ=−k⋅g−c⋅ġ with
// linear dissipation model constant c = τ⋅k.
template <typename T>
class SapCouplerConstraint final : public SapConstraint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapCouplerConstraint);

  // TODO: Simplify, get rid of struct Parameters.
  struct Parameters {
    T gear_ratio{1.0};  // It can be negative to impose opposing motions.
    T stiffness{0.0};
    T dissipation_time_scale{0.0};
    // Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the contact frequency
    // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. w
    // corresponds to a diagonal approximation of the Delassuss operator for
    // each contact. See [Castro et al., 2021. §IX.A] for details.
    double beta{1.0};
  };

  // @throws if the number of rows in J is different from three.
  SapCouplerConstraint(const Parameters& p, int clique, const MatrixX<T>& J,
                       const T& phi0);

  // @throws if the number of rows in J0 and J1 is different from three.
  SapCouplerConstraint(const Parameters& p, int clique0, int clique1,
                       const MatrixX<T>& J0, const MatrixX<T>& J1, const T& g0);

  void Project(const Eigen::Ref<const VectorX<T>>& y,
               const Eigen::Ref<const VectorX<T>>& R,
               EigenPtr<VectorX<T>> gamma,
               MatrixX<T>* dPdy = nullptr) const final;

  VectorX<T> CalcBiasTerm(const T& time_step, const T& wi) const final;
  VectorX<T> CalcDiagonalRegularization(const T& time_step,
                                        const T& wi) const final;

 private:
  Parameters parameters_;
  T g0_;
};

// SAP Problem defined by:
//   - A⋅(v−v*) = Jᵀ⋅γ
//   - Constraints.
// See design document for more details.
template <typename T>
class SapContactProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapContactProblem);

  // Constructs a SAP contact problem where the linearized dynamics read:
  //  A⋅(v−v*) = Jᵀ⋅γ
  // where A is the linearized discrete momentum matrix and v* are the
  // free-motion generalized velocities. The contact Jacobian J is specified as
  // constraints are added with AddConstraint().
  // 
  // @param time_step The time step used by the discrete SAP formulation.
  // @param A Linearized dynamics momentum matrix. It is block diagonal. Each
  // block corresponds to a group of DOFs or "clique". The number of cliques is
  // A.size() and the number of DOFs in the c-th clique is A[c].rows() (or
  // A[c].cols() since each block is square). The total number of generalized
  // velocities is nv = ∑A[c].rows().
  // @param v_star Free-motion velocities, of size nv. DOFs order must
  // correspond to those referenced in A.
  // 
  // TODO: I found useful to keep a reference to the original data in the scope
  // where the problem is built (the manager). Therefore it might be more
  // convenient for the constructor to keep references to data rather than
  // moving copies of the original data.
  SapContactProblem(const T& time_step, std::vector<MatrixX<T>>&& A,
                    VectorX<T>&& v_star);

  // TODO: I believe this is not needed. Remove.
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
