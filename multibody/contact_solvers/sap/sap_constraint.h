#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

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

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake