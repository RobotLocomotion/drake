#pragma once

#include <array>
#include <memory>
#include <utility>

#include <Eigen/Sparse>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/abstract_fem_state.h"
#include "drake/multibody/fixed_fem/dev/dirichlet_boundary_condition.h"

namespace drake {
namespace multibody {
namespace fixed_fem {

/** The abstract base class for FemModel that hides its template parameter. See
 FemModel for more information.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class FemModelBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemModelBase);
  FemModelBase() = default;
  virtual ~FemModelBase() = default;

  /** The order of the ODE problem associated with `this` %FemModel. */
  virtual int ode_order() const = 0;

  /** The number of nodes that are associated with `this` %FemModel. */
  int num_nodes() const { return num_nodes_; }

  /** The number of degrees of freedom in the model. */
  virtual int num_dofs() const = 0;

  /** The number of FemElements owned by `this` %FemModel. */
  virtual int num_elements() const = 0;

  /** Creates a new FemStateBase whose concrete FemState type is compatible
   with the concrete FemModel type of `this` %FemModelBase. The new state's
   number of generalized positions is equal to `num_dofs()`. The new state's
   element data is constructed using the FemElements owned by this %FemModel. */
  virtual std::unique_ptr<FemStateBase<T>> MakeFemStateBase() const = 0;

  /** Calculates the residual at the given FemStateBase. Suppose the linear
  or nonlinear system generated from the FEM discretization is G(z) = 0, then
  the output `residual` is equal to the function G evaluated at the input
  `state`.
  @pre residual != nullptr.
  @throw std::exception if the type of concrete FemState in `state` is not
  compatible with the concrete FemModel in `this` model. */
  void CalcResidual(const FemStateBase<T>& state,
                    EigenPtr<VectorX<T>> residual) const;

  /** Calculates the tangent matrix at the given FemStateBase. The ij-th
   entry of the tangent matrix is the derivative of the i-th entry of the
   residual (calculated by CalcResidual()) with respect to the j-th generalized
   unknown zⱼ.
   @param[in] state    The FemStateBase at which the residual is evaluated.
   @param[out] tangent_matrix    The output tangent_matrix. Suppose the linear
   or nonlinear system generated from the FEM discretization is G(z) = 0, then
   `tangent_matrix` is equal to ∇G evaluated at the input `state`.
   @pre tangent_matrix != nullptr.
   @pre The size of the matrix behind `tangent_matrix` is `num_dofs()` *
   `num_dofs()`.
   @throw std::exception if the type of concrete FemState in `state` is not
   compatible with the concrete FemModel in `this` model. */
  void CalcTangentMatrix(const FemStateBase<T>& state,
                         Eigen::SparseMatrix<T>* tangent_matrix) const;

  /** Sets the sparsity pattern for the tangent matrix of this %FemModel.
   @param[out] tangent_matrix    The tangent matrix of this %FemModel. Its size
   and sparsity pattern will be set so that it will be ready to be passed into
   CalcTangentMatrix().
   @pre `tangent_matrix` must not be the null pointer. */
  void SetTangentMatrixSparsityPattern(
      Eigen::SparseMatrix<T>* tangent_matrix) const;

  /** Updates the FemStateBase `state` given the change in the value of the
   highest order state `z`.
   @pre state != nullptr.
   @pre dz.size() == state.num_generalized_positions().
   @throw std::exception if the type of concrete FemState in `state` is not
   compatible with the concrete FemModel in `this` model. */
  void UpdateState(const VectorX<T>& dz, FemStateBase<T>* state) const;

  // TODO(xuchenhan-tri): Consider renaming this method to imply that the method
  //  provides only a prediction of the state at the next time step.
  /** Predicts the state of the FEM model at the next time step given the state
   at the previous time step. If `this` %FemModel is zeroth_order (see
   ode_order()), throw an exception.
   @param[in] prev_state The state at the previous time step.
   @param[in, out] next_state The predictor state at the new time step.
   @pre next_state != nullptr.
   @pre next_state->num_generalized_positions() ==
   prev_state.num_generalized_positions(). */
  void AdvanceOneTimeStep(const FemStateBase<T>& prev_state,
                          FemStateBase<T>* next_state) const;

  /* Apply boundary condition set for this %FemModel to the input `state`. No-op
   if no boundary condition is set.
   @pre state != nullptr. */
  void ApplyBoundaryConditions(FemStateBase<T>* state) const;

  /** Takes ownership of the given Dirichlet boundary condition and apply it
   when the model is evaluated. */
  void SetDirichletBoundaryCondition(
      std::unique_ptr<DirichletBoundaryCondition<T>> dirichlet_bc) {
    dirichlet_bc_ = std::move(dirichlet_bc);
  }

  /** (Internal use only) Throws std::logic_error to report a mismatch between
  the concrete types of `this` FemModelBase and the FemStateBase that was
  passed to API method `func`. */
  virtual void ThrowIfModelStateIncompatible(
      const char* func, const FemStateBase<T>& state_base) const = 0;

 protected:
  /** Derived classes must override this method to provide an implementation for
   the NVI CalcResidual(). The input `state` is guaranteed to be compatible with
   `this` FEM model. */
  virtual void DoCalcResidual(const FemStateBase<T>& state,
                              EigenPtr<VectorX<T>> residual) const = 0;

  /** Derived classes must override this method to provide an implementation for
   the NVI CalcTangentMatrix(). The input `state` is guaranteed to be compatible
   with `this` FEM model. */
  virtual void DoCalcTangentMatrix(
      const FemStateBase<T>& state,
      Eigen::SparseMatrix<T>* tangent_matrix) const = 0;

  /** Derived classes must override this method to provide an implementation for
   the NVI SetTangentMatrixSparsityPattern(). */
  virtual void DoSetTangentMatrixSparsityPattern(
      Eigen::SparseMatrix<T>* tangent_matrix) const = 0;

  /** Derived classes must override this method to provide an implementation for
   the NVI UpdateState(). The input `state` is guaranteed to be compatible with
   `this` FEM model. */
  virtual void DoUpdateState(const VectorX<T>& dz,
                             FemStateBase<T>* state) const = 0;

  /** Derived classes must override this method to provide an implementation for
   the NVI AdvanceOneTimeStep(). The input `prev_state` and `next_state` are
   guaranteed to be compatible with `this` FEM model. */
  virtual void DoAdvanceOneTimeStep(const FemStateBase<T>& prev_state,
                                    FemStateBase<T>* next_state) const = 0;

  void increment_num_nodes(int num_new_nodes) { num_nodes_ += num_new_nodes; }

 private:
  /* The total number of nodes in the system. */
  int num_nodes_{0};
  /* The Dirichlet boundary condition imposed on the model. */
  std::unique_ptr<DirichletBoundaryCondition<T>> dirichlet_bc_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::FemModelBase);
