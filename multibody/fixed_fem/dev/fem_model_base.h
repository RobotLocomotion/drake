#pragma once

#include <array>
#include <memory>
#include <utility>

#include <Eigen/Sparse>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/dirichlet_boundary_condition.h"
#include "drake/multibody/fixed_fem/dev/fem_state_base.h"

namespace drake {
namespace multibody {
namespace fixed_fem {

/** %FemModel calculates the components of the discretized FEM equations.
 Suppose the PDE at hand is of the form

     F(z, ∇z, ...) = 0.

 where ... indicates possible higher derivatives that we aren't concerned with
 here. In this PDE, z: Ω ⊂ Rᴰ → Rᵈ, is the unknown function being solved for
 (see also UpdateStateFromChangeInUnknowns()). Here, Ω is the domain, D is the
 dimension of the domain, and d is the solution dimension. For instance, if you
 are solving for the temperature of a 3D object, then the domain is
 three-dimensional (D = 3), while the solution, which is the temperature at a
 point within the object, is one-dimensional (d = 1). After spatial and time
 discretization, the PDE is reduced to a system of linear or nonlinear equations
 of the form:

     G(z₁, z₂, ..., zₙ) = 0,

 where n is the number of nodes in the discretization and G is a function from
 Rⁿᵈ to Rⁿᵈ. The linear or nonlinear equation in the system associated with
 the node `a` has the form

     Gₐ(z₁, z₂, ..., zₙ) = 0,

 where Gₐ is a function from Rⁿᵈ → Rᵈ and a = 1, ..., n. The nodal values z₁,
 z₂, ..., zₙ are solved for with a linear or nonlinear solver, and the solution
 z is interpolated from these nodal values.

 %FemModel calculates various components of the system of linear or nonlinear
 equations that supports solving the system. For example, CalcResidual()
 calculates the value of G evaluated at the current state and
 CalcTangentMatrix() calculates ∇G at the current state.
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
  std::unique_ptr<FemStateBase<T>> MakeFemStateBase() const;

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

  /** Updates the FemStateBase `state` given the change in the unknown variable
   `dz`.
   @pre state != nullptr.
   @pre dz.size() == state->num_generalized_positions().
   @throw std::exception if the type of concrete FemState for `state` is not
   compatible with the concrete FemModel for `this` model. */
  void UpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                       FemStateBase<T>* state) const;

  /** For a dynamic FEM model, calculates the state at the next time step given
   the state at the previous time step and the highest order state at the next
   time step. If `this` %FemModel is static (ode_order() == 0), throw an
   exception.
   @param[in] prev_state The state at the previous time step.
   @param[in] highest_order_state The highest order state at the next time step.
   @param[out] next_state The state at the next time step.
   @pre next_state != nullptr.
   @pre next_state->num_generalized_positions() ==
   prev_state.num_generalized_positions().
   @pre next_state->num_generalized_positions() == highest_order_state.size().
   @throw std::exception if the type of concrete FemState in `prev_state` or
   `next_state` is not compatible with the concrete FemModel in `this` model.
   @throw std::exception if ode_order() == 0. */
  void AdvanceOneTimeStep(const FemStateBase<T>& prev_state,
                          const VectorX<T>& highest_order_state,
                          FemStateBase<T>* next_state) const;

  /* Apply boundary condition set for this %FemModel to the input `state`. No-op
   if no boundary condition is set.
   @pre state != nullptr. */
  void ApplyBoundaryCondition(FemStateBase<T>* state) const;

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
    the NVI MakeFemStateBase(). */
  virtual std::unique_ptr<FemStateBase<T>> DoMakeFemStateBase() const = 0;

  /** Derived classes must override this method to provide an implementation
   for the NVI CalcResidual(). The input `state` is guaranteed to be
   compatible with `this` FEM model. */
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
   the NVI UpdateStateFromChangeInUnknowns(). The input `state` is
   guaranteed to be compatible with `this` FEM model. */
  virtual void DoUpdateStateFromChangeInUnknowns(
      const VectorX<T>& dz, FemStateBase<T>* state) const = 0;

  /** Derived classes must override this method to provide an implementation for
   the NVI AdvanceOneTimeStep(). The input `prev_state` and `next_state` are
   guaranteed to be compatible with `this` FEM model. The size of `prev_state`,
   `highest_order_state`, and `next_state` are guaranteed to be compatible. */
  virtual void DoAdvanceOneTimeStep(const FemStateBase<T>& prev_state,
                                    const VectorX<T>& highest_order_state,
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
