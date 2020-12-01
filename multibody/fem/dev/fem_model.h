#pragma once

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/fem_element.h"
#include "drake/multibody/fem/dev/fem_indexes.h"
#include "drake/multibody/fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {
/** %FemModel calculates the components of the discretized FEM equations.
 Suppose the PDE at hand is of the form

     F(u, ∇u, ...) = 0.

 where ... indicates possible higher derivatives that we aren't concerned with
 here. In this PDE, u: Ω ⊂ Rᴰ → Rᵈ, is the unknown function being solved for.
 Here, Ω is the domain, D is the dimension of the domain, and d is the solution
 dimension. For instance, if you are solving for the temperature of a 3D object,
 then the domain is three-dimensional (D = 3), while the solution, which is the
 temperature at a point within the object, is one-dimensional (d = 1). After the
 FEM discretization, the PDE is reduced to a system of linear or nonlinear
 equations of the form:

     𝐆(u₁, u₂, ..., uₙ) = 0,

 where n is the number of nodes in the discretization and 𝐆 is a function from
 Rⁿᵈ to Rⁿᵈ. The linear or nonlinear equation in the system associated with
 the node `a` has the form

     Gₐ(u₁, u₂, ..., uₙ) = 0,

 where Gₐ is a function from Rⁿᵈ → Rᵈ and a = 1, ..., n. The nodal values u₁,
 u₂, ..., uₙ are solved for with a linear or nonlinear solver, and the solution
 u is interpolated from these nodal values.

 %FemModel calculates various components of the system of linear or nonlinear
 equations that supports solving the system. For example, CalcResidual()
 calculates the value of 𝐆 evaluated at the current state and
 CalcTangentMatrix() calculates ∇𝐆 at the current state.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class FemModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemModel);

  FemModel() = default;

  virtual ~FemModel() = default;

  /** Creates a new FemState. The new state's number of generalized positions is
   equal to `num_dofs()`. The new state's element cache is
   compatible with the FemElement's owned by this %FemModel. */
  std::unique_ptr<FemState<T>> MakeFemState() const;

  /** The dimension of the codomain of the PDE's solution u. */
  virtual int solution_dimension() const = 0;

  /** The number of degrees of freedom in the model. It is equal to
   `solution_dimension()` * `num_nodes()`. */
  int num_dofs() const { return solution_dimension() * num_nodes(); }

  /** The number of FemElement's owned by `this` %FemModel. */
  int num_elements() const { return elements_.size(); }

  /** The number of nodes that are associated with `this` %FemModel. */
  int num_nodes() const { return num_nodes_; }

  /** Calculates the residual at the given FemState. Suppose the linear or
   nonlinear system generated from the FEM discretization is 𝐆(𝐮) = 0,
   then the output `residual` is equal to the function 𝐆 evaluated at the
   input `state`.
   @param[in] state The FemState at which to evaluate the residual.
   @param[out] residual The output residual evaluated at `state`.
   @pre The size of `residual` must be equal to `num_dofs()`. */
  void CalcResidual(const FemState<T>& state,
                    EigenPtr<VectorX<T>> residual) const;

  /** Calculates the tangent matrix at the given FemState. The ij-th entry of
   the tangent matrix is the derivative of the i-th entry of the residual
   (calculated by CalcResidual()) with respect to the j-th generalized unknown
   uⱼ.
   @param[in] state The FemState to evaluate the residual at.
   @param[out] tangent_matrix The output tangent_matrix. Suppose the linear or
   nonlinear system generated from the FEM discretization is 𝐆(𝐮) = 0,
   then `tangent_matrix` is equal to ∇𝐆 evaluated at the input `state`.
   @pre `tangent_matrix` must not be the null pointer.
   @pre The size of matrix pointed to by `tangent_matrix` must be
   `num_dofs()`*`num_dofs()`. */
  void CalcTangentMatrix(const FemState<T>& state,
                         Eigen::SparseMatrix<T>* tangent_matrix) const;

  /** Sets the sparsity pattern for the tangent matrix of this %FemModel.
    @param[out] tangent_matrix The tangent matrix of this %FemModel. Its
    sparsity pattern will be set and it will be ready to be passed into
    CalcTangentMatrix.
    @pre `tangent_matrix` must not be the null pointer. */
  void SetTangentMatrixSparsityPattern(
      Eigen::SparseMatrix<T>* tangent_matrix) const;

  // TODO(xuchenhan-tri): Add missing method: CalcMassMatrix.

 protected:
  /** Derived class should create a new FemState with no element cache and
   initialize the per-node states. */
  virtual std::unique_ptr<FemState<T>> DoMakeFemState() const = 0;

  /** Throws if the `node_indices` are not consecutive from 0 to
   the `node_indices.size()` - 1. */
  void ThrowIfNodeIndicesAreInvalid(const std::set<int>& node_indices) const;

  /** Takes ownership of the input `element` and adds it into `this` FemModel.
   */
  void AddElement(std::unique_ptr<FemElement<T>> element);

  const FemElement<T>& element(ElementIndex i) const {
    DRAKE_ASSERT(i.is_valid());
    DRAKE_ASSERT(i < num_elements());
    return *elements_[i];
  }

  /** Increment num_nodes by `num_new_nodes`. */
  void increment_num_nodes(int num_new_nodes) { num_nodes_ += num_new_nodes; }

 private:
  /* FemElements owned by this model. */
  std::vector<std::unique_ptr<FemElement<T>>> elements_;
  /* The total number of nodes in the system. */
  int num_nodes_{0};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemModel);
