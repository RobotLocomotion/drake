#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** %FemElement is the base class for spatially discretized FEM elements.
 It computes quantities such as the residual and the stiffness matrix on a
 single FEM element given the state of the FEM system. These quantities are then
 assembled into their global counterparts by FemModel.

 %FemElement serves as the interface base class for all FEM elements.
 Since FEM elements are usually evaluated in computationally intensive inner
 loops of the simulation, the overhead caused by virtual methods and heap
 allocations may be significant. Therefore, this class uses a CRTP pattern to
 achieve compile-time polymorphism and avoids the overhead of virtual methods
 and facilitates inlining instead. The type information at compile time also
 helps eliminate all heap allocations. Derived FEM elements models must inherit
 from this base class and implement the interface this class provides. The
 derived FEM elements must also be accompanied by a corresponding traits class
 that declares the compile time quantities and type declarations that this base
 class requires.

 %FemElement should be used in tandem with ElementCacheEntry. There
 should be a one-to-one correspondence between each %FemElement that
 performs the element routine and each ElementCacheEntry that stores the
 state-dependent quantities used in the routine. This correspondence is
 maintained by the shared element index that is assigned to both the
 %FemElement and the ElementCacheEntry in correspondence. Furthermore,
 the type of %FemElement and ElementCacheEntry in correspondence must
 be compatible. More specifically, if the %FemElement is of concrete
 type `FooElement`, then the ElementCacheEntry that shares the same element
 index must be of concrete type `FooElementCacheEntry`.
 @tparam DerivedElement The concrete FEM element that inherits from %FemElement
 through CRTP.
 @tparam DerivedTraits The traits class associated with the DerivedElement. */
template <class DerivedElement, class DerivedTraits>
class FemElement {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemElement);

  using T = typename DerivedTraits::T;

  /** The type of ElementcacheEntry that this compatible with this %FemElement.
   */
  using ElementCacheEntryType = typename DerivedTraits::ElementCacheEntryType;

  /** The dimension of the parent domain for this element. */
  static constexpr int natural_dimension() {
    return DerivedTraits::kNaturalDimension;
  }

  /** The dimension of the codomain of the unknown function u. */
  static constexpr int solution_dimension() {
    return DerivedTraits::kSolutionDimension;
  }

  /** The dimension of the reference domain for this element. */
  static constexpr int spatial_dimension() {
    return DerivedTraits::kSpatialDimension;
  }

  /** Number of quadrature points in this element at which various quantities
   * are evaluated. */
  static constexpr int num_quadrature_points() {
    return DerivedTraits::kNumQuadraturePoints;
  }

  /** Number of nodes associated with this element. */
  static constexpr int num_nodes() { return DerivedTraits::kNumNodes; }

  /** Number of degrees of freedom associated with this element. */
  static constexpr int num_dofs() { return DerivedTraits::kNumDofs; }

  /** The order of the ODE problem after FEM spatial discretization. */
  static constexpr int ode_order() { return DerivedTraits::kODEOrder; }

  /** Global indices of the nodes of this element. */
  const std::array<NodeIndex, num_nodes()>& node_indices() const {
    return node_indices_;
  }

  /** Global ElementIndex of this element. */
  ElementIndex element_index() const { return element_index_; }

  /** Creates an ElasticityElementCacheEntry that is compatible with this
   * element. */
  ElementCacheEntryType MakeElementCacheEntry() const {
    return ElementCacheEntryType(element_index());
  }

  /** Calculates the element residual of this element evaluated at the input
   state.
   @param[in] state The FEM state at which to evaluate the residual.
   @param[out] a vector of residual of size `num_dofs()`. */
  void CalcResidual(const FemState<DerivedElement>& state,
                    EigenPtr<Vector<T, num_dofs()>> residual) const {
    DRAKE_ASSERT(residual != nullptr);
    residual->setZero();
    static_cast<const DerivedElement*>(this)->DoCalcResidual(state, residual);
  }

  /** Calculates the stiffness matrix (the derivative of the residual with
  respect to the generalized positions) of this element given the state.
  @param[in] state The FEM state at which to evaluate the stiffness matrix.
  @param[out] K The stiffness matrix of size `num_dofs()`-by-`num_dofs()`. */
  void CalcStiffnessMatrix(
      const FemState<DerivedElement>& state,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> K) const {
    DRAKE_ASSERT(K != nullptr);
    K->setZero();
    static_cast<const DerivedElement*>(this)->DoCalcStiffnessMatrix(state, K);
  }

  /** Calculates the damping matrix (the derivative of the residual with respect
  to the time derivative of generalized positions) of this element given the
  state.
  @param[in] state The FEM state at which to evaluate the damping matrix.
  @param[out] D The damping matrix of size `num_dofs()`-by-`num_dofs()`. */
  void CalcDampingMatrix(
      const FemState<DerivedElement>& state,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> D) const {
    DRAKE_ASSERT(D != nullptr);
    D->setZero();
    static_cast<const DerivedElement*>(this)->DoCalcDampingMatrix(state, D);
  }

  /** Calculates the mass matrix (the derivative of the residual with respect to
  the time second derivative of generalized positions) of this element given the
  state.
  @param[in] state The FEM state at which to evaluate the mass matrix.
  @param[out] M The mass matrix of size `num_dofs()`-by-`num_dofs()`. */
  void CalcMassMatrix(
      const FemState<DerivedElement>& state,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> M) const {
    DRAKE_ASSERT(M != nullptr);
    M->setZero();
    static_cast<const DerivedElement*>(this)->DoCalcMassMatrix(state, M);
  }

 protected:
  /** Constructs a new FEM element.
   @param[in] element_index The global index of the new element.
   @param[in] node_indices The global node indices of the nodes of this
   element.
   @pre element_index must be valid. */
  FemElement(ElementIndex element_index,
             const std::array<NodeIndex, num_nodes()>& node_indices)
      : element_index_(element_index), node_indices_(node_indices) {}

 private:
  // The global index of this element.
  ElementIndex element_index_;
  // The global node indices of this element.
  std::array<NodeIndex, num_nodes()> node_indices_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
