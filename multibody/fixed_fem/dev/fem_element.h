#pragma once

#include <array>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {
// TODO(xuchenhan-tri) Document the definition of quantities like "natural
// dimension". See issue #14475.
/** %FemElement is the base class for spatially discretized FEM elements.
 It computes quantities such as the residual and the stiffness matrix on a
 single FEM element given the state of the FEM system. These quantities are then
 assembled into their global counterparts by FemModel.

 %FemElement serves as the base class for all FEM elements. Since FEM elements
 are usually evaluated in computationally intensive inner loops of the
 simulation, the overhead caused by virtual methods and heap allocations may be
 significant. Therefore, this class uses CRTP to achieve compile-time
 polymorphism and avoids the overhead of virtual methods and facilitates
 inlining instead. The type information at compile time also helps eliminate all
 heap allocations. Derived FEM elements must inherit from this base class and
 implement the interface this class provides. The derived FEM elements must also
 be accompanied by a corresponding traits class that declares the compile time
 quantities and type declarations that this base class requires.

 %FemElement also comes with per-element, state-dependent data. The data
 specific to the `DerivedElement` should be declared in `DerivedTraits`, along
 with the other responsibilities of `DerivedTraits` detailed below.
 @tparam DerivedElement The concrete FEM element that inherits from %FemElement
 through CRTP.
 @tparam DerivedTraits The traits class associated with the DerivedElement. It
 needs to provide a nested class `Data` for storing the per-element,
 state-dependent data used by `DerivedElement`. In particular, the `Data` class
 needs to provide a default constructor. It also needs to provide the following
 compile time constants for the `DerivedElement`: `kNaturalDimension`,
 `kSolutionDimension`, `kSpatialDimension`, the number of quadrature points in a
 single `DerivedElement`, `kNumQuadraturePoints`, the number of nodes associated
 with a single `DerivedElement`, `kNumNodes`, the number of degrees of freedom
 that a single `DerivedElement` possesses, `kNumDofs`, and the order of the ODE
 problem after FEM spatial discretization, `kOdeOrder`. */
template <class DerivedElement, class DerivedTraits>
class FemElement {
 public:
  using T = typename DerivedTraits::T;
  using Traits = DerivedTraits;

  /** Indices of the nodes of this element within the model. */
  const std::array<NodeIndex, Traits::kNumNodes>& node_indices() const {
    return node_indices_;
  }

  /** The ElementIndex of this element within the model. */
  ElementIndex element_index() const { return element_index_; }

  /** Computes the per-element, state-dependent data associated with this
   `DerivedElement` given the `state`.
   @pre data != nullptr. */
  typename Traits::Data ComputeData(
      const FemState<DerivedElement>& state) const {
    return static_cast<const DerivedElement*>(this)->DoComputeData(state);
  }

  /** Calculates the element residual of this element evaluated at the input
   state.
   @param[in] state    The FEM state at which to evaluate the residual.
   @param[out] residual    A vector of residual of size `Traits::kNumDofs`. All
   values in `residual` will be overwritten.
   @pre residual != nullptr */
  void CalcResidual(const FemState<DerivedElement>& state,
                    EigenPtr<Vector<T, Traits::kNumDofs>> residual) const {
    DRAKE_ASSERT(residual != nullptr);
    residual->setZero();
    static_cast<const DerivedElement*>(this)->DoCalcResidual(state, residual);
  }

  /** Calculates the stiffness matrix (the derivative, or an approximation
  thereof, of the residual with respect to the generalized positions) of this
  element given the state.
  @param[in] state    The FEM state at which to evaluate the stiffness matrix.
  @param[out] K    The stiffness matrix of size
  `Traits::kNumDofs`-by-`Traits::kNumDofs`. All values of `K` will be
  overwritten.
  @pre K != nullptr */
  void CalcStiffnessMatrix(
      const FemState<DerivedElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> K) const {
    DRAKE_ASSERT(K != nullptr);
    K->setZero();
    static_cast<const DerivedElement*>(this)->DoCalcStiffnessMatrix(state, K);
  }

  /** Calculates the damping matrix (the derivative of the residual with respect
  to the time derivative of generalized positions) of this element given the
  state.
  @param[in] state    The FEM state at which to evaluate the damping matrix.
  @param[out] D    The damping matrix of size
  `Traits::kNumDofs`-by-`Traits::kNumDofs`. All values of `D` will be
  overwritten.
  @pre D != nullptr */
  void CalcDampingMatrix(
      const FemState<DerivedElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> D) const {
    DRAKE_ASSERT(D != nullptr);
    D->setZero();
    static_cast<const DerivedElement*>(this)->DoCalcDampingMatrix(state, D);
  }

  /** Calculates the mass matrix (the derivative of the residual with respect to
  the time second derivative of generalized positions) of this element given the
  state.
  @param[in] state    The FEM state at which to evaluate the mass matrix.
  @param[out] M    The mass matrix of size
  `Traits::kNumDofs`-by-`Traits::kNumDofs`. All values of `M` will be
  overwritten.
  @pre M != nullptr */
  void CalcMassMatrix(
      const FemState<DerivedElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> M) const {
    DRAKE_ASSERT(M != nullptr);
    static_cast<const DerivedElement*>(this)->DoCalcMassMatrix(state, M);
  }

  /** Extract the dofs corresponding to the nodes given by `node_indices` from
   the given `state_dofs`. */
  static Vector<T, Traits::kSolutionDimension * Traits::kNumNodes>
  ExtractElementDofs(
      const std::array<NodeIndex, Traits::kNumNodes>& node_indices,
      const VectorX<T>& state_dofs) {
    constexpr int kDim = Traits::kSolutionDimension;
    Vector<T, kDim * Traits::kNumNodes> element_dofs;
    for (int i = 0; i < Traits::kNumNodes; ++i) {
      DRAKE_ASSERT((node_indices[i] + 1) * kDim <= state_dofs.size());
      element_dofs.template segment<kDim>(i * kDim) =
          state_dofs.template segment<kDim>(node_indices[i] * kDim);
    }
    return element_dofs;
  }

 protected:
  /** Assignment and copy constructions are prohibited.
   Move constructor is allowed so that FemElement can be stored in
   `std::vector`. */
  FemElement(const FemElement&) = delete;
  FemElement(FemElement&&) = default;
  const FemElement& operator=(const FemElement&) = delete;
  FemElement&& operator=(const FemElement&&) = delete;

  /** Constructs a new FEM element. The constructor is made protected because
   FemElement should not be constructed directly. Use the constructor of the
   derived classes instead.
   @param[in] element_index    The index of the new element within the model.
   @param[in] node_indices    The node indices of the nodes of this element
   within the model.
   @pre element_index is valid.
   @pre Entries in node_indices are valid. */
  FemElement(ElementIndex element_index,
             const std::array<NodeIndex, Traits::kNumNodes>& node_indices)
      : element_index_(element_index), node_indices_(node_indices) {
    DRAKE_ASSERT(element_index.is_valid());
    for (int i = 0; i < Traits::kNumNodes; ++i) {
      DRAKE_ASSERT(node_indices[i].is_valid());
    }
  }

  /** `DerivedElement` must provide an implementation for `DoComputeData()`.
   @throw std::exception if `DerivedElement` does not provide an implementation
   for `DoComputeData()`. */
  typename Traits::Data DoComputeData(
      const FemState<DerivedElement>& state) const {
    ThrowIfNotImplemented(__func__);
  }

  /** `DerivedElement` must provide an implementation for
   `DoCalcResidual()` to provide the residual that is up to date given the
   `state`. The caller guarantees that `residual` is non-null and contains all
   zeros; the implementation in the derived class does not have to test for
   this.
   @throw std::exception if `DerivedElement` does not provide an implementation
   for `DoCalcResidual()`. */
  void DoCalcResidual(const FemState<DerivedElement>& state,
                      EigenPtr<Vector<T, Traits::kNumDofs>> residual) const {
    ThrowIfNotImplemented(__func__);
  }

  /** `DerivedElement` must provide an implementation for
   `DoCalcStiffnessMatrix()` to provide the stiffness matrix that is up to date
   given the `state`. The caller guarantees that `K` is non-null and contains
   all zeros; the implementation in the derived class does not have to test for
   this.
   @throw std::exception if `DerivedElement` does not provide an implementation
   for `DoCalcStiffnessMatrix()`. */
  void DoCalcStiffnessMatrix(
      const FemState<DerivedElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> K) const {
    ThrowIfNotImplemented(__func__);
  }

  /** `DerivedElement` must provide an implementation for
   `DoCalcDampingMatrix()` to provide the damping matrix that is up to date
   given the `state` or to throw an exception indicating a damping matrix does
   not exist. The caller guarantees that `D` is non-null and contains all
   zeros; the implementation in the derived class does not have to test for
   this.
   @throw std::exception if `DerivedElement` does not provide an implementation
   for `DoCalcDampingMatrix()`. */
  void DoCalcDampingMatrix(
      const FemState<DerivedElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> D) const {
    ThrowIfNotImplemented(__func__);
  }

  /** `DerivedElement` must provide an implementation for `DoCalcMassMatrix()`
   to provide the mass matrix that is up-to-date given the `state` or to throw
   an exception indicating a mass matrix does not exist. The caller guarantees
   that `M` is non-null (but it is uninitialized); the implementation in the
   derived class does not have to test for this.
   @throw std::exception if `DerivedElement` does not provide an implementation
   for `DoCalcMassMatrix()`. */
  void DoCalcMassMatrix(
      const FemState<DerivedElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> M) const {
    ThrowIfNotImplemented(__func__);
  }

 private:
  /* Helper to throw a descriptive exception when a given function is not
   implemented. */
  void ThrowIfNotImplemented(const char* source_method) const {
    throw std::runtime_error("The DerivedElement from " +
                             NiceTypeName::Get(*this) +
                             " must provide an implementation for " +
                             std::string(source_method) + "().");
  }

  /* The index of this element within the model. */
  ElementIndex element_index_;
  /* The node indices of this element within the model. */
  std::array<NodeIndex, Traits::kNumNodes> node_indices_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
