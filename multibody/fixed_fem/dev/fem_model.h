#pragma once

#include <type_traits>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/fem_element.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** %FemModel calculates the components of the discretized FEM equations.
 Suppose the PDE at hand is of the form

     F(z, ∇z, ...) = 0.

 where ... indicates possible higher derivatives that we aren't concerned with
 here. In this PDE, z: Ω ⊂ Rᴰ → Rᵈ, is the unknown function being solved for.
 Here, Ω is the domain, D is the dimension of the domain, and d is the solution
 dimension. For instance, if you are solving for the temperature of a 3D object,
 then the domain is three-dimensional (D = 3), while the solution, which is the
 temperature at a point within the object, is one-dimensional (d = 1). After
 spatial and time discretization, the PDE is reduced to a system of linear or
 nonlinear equations of the form:

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
 @tparam Element The type of FemElement that makes up this %FemModel.
 This template parameter provides the scalar type and the compile time constants
 such as the natural and spatial dimension and the number of nodes in each
 element. */
template <class Element>
class FemModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemModel);

  static_assert(
      std::is_base_of_v<FemElement<Element, typename Element::Traits>, Element>,
      "The template parameter Element should be derived from FemElement. ");
  using T = typename Element::Traits::T;

  /** Creates a new FemState. The new state's number of generalized positions is
   equal to `num_dofs()`. The new state's element data is constructed using the
   FemElements owned by this %FemModel. */
  FemState<Element> MakeFemState() const;

  /** The number of degrees of freedom in the model. */
  int num_dofs() const {
    return Element::Traits::kSolutionDimension * num_nodes();
  }

  /** The number of FemElements owned by `this` %FemModel. */
  int num_elements() const { return elements_.size(); }

  /** The number of nodes that are associated with `this` %FemModel. */
  int num_nodes() const { return num_nodes_; }

  /** Calculates the residual at the given FemState by assembling the residual
   from each element. Suppose the linear or nonlinear system generated from the
   FEM discretization is G(z) = 0, then the output `residual` is equal to the
   function G evaluated at the input `state`.
   @param[in] state The FemState at which to evaluate the residual.
   @param[out] residual The output residual evaluated at `state`.
   @pre residual != nullptr.
   @pre The size of the vector behind `residual` must be `num_dofs()`. */
  void CalcResidual(const FemState<Element>& state,
                    EigenPtr<VectorX<T>> residual) const;

  /** Calculates the tangent matrix at the given FemState. The ij-th entry of
   the tangent matrix is the derivative of the i-th entry of the residual
   (calculated by CalcResidual()) with respect to the j-th generalized unknown
   zⱼ.
   @param[in] state    The FemState at which the residual is evaluated.
   @param[in] updater    The StateUpdater that updates the FemState given the
   solution to the system G(z) = 0.
   @param[out] tangent_matrix    The output tangent_matrix. Suppose the linear
   or nonlinear system generated from the FEM discretization is G(z) = 0, then
   `tangent_matrix` is equal to ∇G evaluated at the input `state`.
   @pre tangent_matrix != nullptr.
   @pre The size of the matrix behind `tangent_matrix` is `num_dofs()` *
   `num_dofs()`. */
  void CalcTangentMatrix(const FemState<Element>& state,
                         const StateUpdater& updater,
                         Eigen::SparseMatrix<T>* tangent_matrix) const;

 protected:
  FemModel() = default;
  ~FemModel() = default;

  /** Derived classes must implement this method to create a new FemState. */
  virtual FemState<Element> DoMakeFemState() const = 0;

  void AddElement(Element&& element) {
    elements_.emplace_back(std::move(element)) j;
  }

  void increment_num_nodes(int num_new_nodes) { num_nodes_ += num_new_nodes; }

 private:
  /* Builds the element tangent matrix for the element with index
   `element_index` by combining the stiffness matrix, damping matrix, and the
   mass matrix according to the weights provided by the `updater`.
   @param[in] state    The FemState to evaluate the residual at.
   @param[in] updater    The StateUpdater that updates the FemState given the
   solution to the system G(z) = 0.
   @param[in] element_index    Index of the element whose element tangent matrix
   is being built. */
  Eigen::Matrix<T, Element::Traits::kNumDofs, Element::Traits::kNumDofs>
  CalcElementTangentMatrix(const FemState<Element>& state,
                           const StateUpdater& updater,
                           ElementIndex element_index) const {
    DRAKE_DEMAND(element_index.is_valid() && element_index < num_elements());
    const Vector3<T> state_derivatives = updater.state_derivatives();
    auto stiffness_matrix = Eigen::Matrix<T, Element::Traits::kNumDofs,
                                          Element::Traits::kNumDofs>::Zero();
    elements_[element_index].CalcStiffnessMatrix(state, &tangent_matrix);
    if constexpr (State::ode_order() == 0) {
      return state_derivatives[0] * stiffness_matrix;
    } else {
      auto damping_matrix = Eigen::Matrix<T, Element::Traits::kNumDofs,
                                          Element::Traits::kNumDofs>::Zero();
      elements_[element_index].CalcDampingMatrix(state, &damping_matrix);
      if constexpr (State::ode_order() == 1) {
        return state_derivatives[0] * stiffness_matrix +
               state_derivatives[1] * damping_matrix;
      } else {
        static_assert(State::ode_order() == 2);
        auto mass_matrix = Eigen::Matrix<T, Element::Traits::kNumDofs,
                                         Element::Traits::kNumDofs>::Zero();
        elements_[element_index].CalcMassMatrix(state, &mass_matrix);
        return state_derivatives[0] * stiffness_matrix +
               state_derivatives[1] * damping_matrix +
               state_derivatives[2] * mass_matrix;
      }
    }
  }

  /* FemElements owned by this model. */
  std::vector<Element> elements_{};
  /* The total number of nodes in the system. */
  int num_nodes_{0};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
