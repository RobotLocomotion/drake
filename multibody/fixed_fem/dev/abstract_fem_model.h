#pragma once

#include <array>
#include <memory>
#include <utility>

#include <Eigen/Sparse>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/abstract_fem_state.h"

namespace drake {
namespace multibody {
namespace fixed_fem {

/** The abstract base class for FemModel that hides its template parameter. See
 FemModel for more information.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class AbstractFemModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AbstractFemModel);
  AbstractFemModel() = default;
  virtual ~AbstractFemModel() = default;

  /** The number of nodes that are associated with `this` %FemModel. */
  int num_nodes() const { return num_nodes_; }

  /** The number of degrees of freedom in the model. */
  virtual int num_dofs() const = 0;

  /** The number of FemElements owned by `this` %FemModel. */
  virtual int num_elements() const = 0;

  /** Creates a new AbstractFemState whose concrete FemState type is compatible
   with the concrete FemModel type of `this` %AbstractFemModel. The new state's
   number of generalized positions is equal to `num_dofs()`. The new state's
   element data is constructed using the FemElements owned by this %FemModel. */
  virtual std::unique_ptr<AbstractFemState<T>> MakeAbstractFemState() const = 0;

  /** Calculates the residual at the given AbstractFemState. Suppose the linear
  or nonlinear system generated from the FEM discretization is G(z) = 0, then
  the output `residual` is equal to the function G evaluated at the input
  `state`.
  @pre residual != nullptr.
  @throw std::exception if the type of concrete FemState in `state` is not
  compatible with the concrete FemModel in `this` model. */
  void CalcResidual(const AbstractFemState<T>& state,
                    EigenPtr<VectorX<T>> residual) const;

  /** Calculates the tangent matrix at the given AbstractFemState. The ij-th
   entry of the tangent matrix is the derivative of the i-th entry of the
   residual (calculated by CalcResidual()) with respect to the j-th generalized
   unknown zⱼ.
   @param[in] state    The AbstractFemState at which the residual is evaluated.
   @param[out] tangent_matrix    The output tangent_matrix. Suppose the linear
   or nonlinear system generated from the FEM discretization is G(z) = 0, then
   `tangent_matrix` is equal to ∇G evaluated at the input `state`.
   @pre tangent_matrix != nullptr.
   @pre The size of the matrix behind `tangent_matrix` is `num_dofs()` *
   `num_dofs()`.
   @throw std::exception if the type of concrete FemState in `state` is not
   compatible with the concrete FemModel in `this` model. */
  void CalcTangentMatrix(const AbstractFemState<T>& state,
                         Eigen::SparseMatrix<T>* tangent_matrix) const;

  /** Sets the sparsity pattern for the tangent matrix of this %FemModel.
   @param[out] tangent_matrix    The tangent matrix of this %FemModel. Its size
   and sparsity pattern will be set so that it will be ready to be passed into
   CalcTangentMatrix().
   @pre `tangent_matrix` must not be the null pointer. */
  void SetTangentMatrixSparsityPattern(
      Eigen::SparseMatrix<T>* tangent_matrix) const;

  /** Updates the AbstractFemState `state` given the change in the value of the
   highest order state `z`.
   @pre state != nullptr.
   @pre dz.size() == state.num_generalized_positions().
   @throw std::exception if the type of concrete FemState in `state` is not
   compatible with the concrete FemModel in `this` model. */
  void UpdateState(const VectorX<T>& dz, AbstractFemState<T>* state) const;

  /** Advance the given `state` by one time step.
   If `this` FemModel is zeroth_order (see FemState::ode_order()), throw an
   exception.
   @param[in] prev_state The state at the previous time step.
   @param[in, out] next_state The state at the new time step.
   @pre next_state != nullptr.
   @pre next_state->num_generalized_positions() ==
   prev_state.num_generalized_positions(). */
  void AdvanceOneTimeStep(const AbstractFemState<T>& prev_state,
                          AbstractFemState<T>* next_state) const;

  /* Apply boundary condition set for this %FemModel to the input `state`. No-op
   if no boundary condition is set.
   @pre state != nullptr. */
  void ApplyBoundaryConditions(AbstractFemState<T>* state) const;

 protected:
  /* Derived classes must override this method to provide an implementation for
   the NVI CalcResidual(). */
  virtual void DoCalcResidual(const AbstractFemState<T>& state,
                              EigenPtr<VectorX<T>> residual) const = 0;

  /* Derived classes must override this method to provide an implementation for
   the NVI CalcTangentMatrix(). */
  virtual void DoCalcTangentMatrix(
      const AbstractFemState<T>& state,
      Eigen::SparseMatrix<T>* tangent_matrix) const = 0;

  /* Derived classes must override this method to provide an implementation for
   the NVI SetTangentMatrixSparsityPattern(). */
  virtual void DoSetTangentMatrixSparsityPattern(
      Eigen::SparseMatrix<T>* tangent_matrix) const = 0;

  /* Derived classes must override this method to provide an implementation for
   the NVI UpdateState(). */
  virtual void DoUpdateState(const VectorX<T>& dz,
                             AbstractFemState<T>* state) const = 0;

  /* Derived classes must override this method to provide an implementation for
   the NVI AdvanceOneTimeStep(). */
  virtual void DoAdvanceOneTimeStep(const AbstractFemState<T>& prev_state,
                                    AbstractFemState<T>* next_state) const = 0;

  /* Derived classes must override this method to provide an implementation for
   the NVI ApplyBoundaryConditions(). */
  virtual void DoApplyBoundaryConditions(AbstractFemState<T>* state) const = 0;

 protected:
  void increment_num_nodes(int num_new_nodes) { num_nodes_ += num_new_nodes; }

 private:
  /* The total number of nodes in the system. */
  int num_nodes_{0};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::AbstractFemModel);
