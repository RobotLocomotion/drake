#pragma once

#include <array>
#include <string>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/constitutive_model.h"
#include "drake/multibody/fem/damping_model.h"
#include "drake/multibody/fem/fem_indexes.h"
#include "drake/multibody/fem/fem_plant_data.h"
#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

// TODO(xuchenhan-tri) Document the definition of quantities like "natural
//  dimension". See issue #14475.

/* Traits class for FemElement(see below). Specializations of concrete traits
 classes must at least define the fields listed in the example below.

   template <>
   struct FemElementTraits<ExampleElement> {
     // Define the scalar type.
     using T = ...;
     // Define element data for the concrete element.
     struct Data {
       // The Data class must be default constructible.
       Data() {...}
       ...
     };
     // The number of quadrature points in each element.
     static constexpr int num_quadrature_points = ...;
     // The natural dimension of the element.
     static constexpr int natural_dimension = ...;
     // The number of nodes in the element.
     static constexpr int num_nodes = ...;
     // The number of degrees of freedom in the element.
     static constexpr int num_dofs = ...;
     // The constitutive model used in the element.
     using ConstitutiveModel = ...;
   };

 @tparam Element The concrete FEM element that inherits from FemElement
 through CRTP. */
template <class Element>
struct FemElementTraits {};

/* FemElement is the base class for spatially discretized FEM elements for
 dynamic elasticity problems. It computes quantities such as the residual and
 the tangent matrix on a single FEM element given the data needed for the
 element's computation. These quantities are then assembled into their global
 counterparts by FemModel. The per-element, state-dependent data used in the
 computation of `DerivedElement` should be declared in the traits, along with
 the other responsibilities of the traits class detailed above.

 Since FEM elements are usually evaluated in computationally intensive inner
 loops of the simulation, the overhead caused by virtual methods and heap
 allocations may be significant. Therefore, this class uses CRTP to achieve
 compile-time polymorphism and avoids the overhead of virtual methods and
 facilitates inlining instead. The type information at compile time also helps
 eliminate all heap allocations. Derived FEM elements must inherit from this
 base class and implement the interface this class provides. The derived FEM
 elements must also be accompanied by a corresponding traits class that
 specializes FemElementTraits (see above).

 @tparam DerivedElement The concrete FEM element that inherits from FemElement
 through CRTP. */
template <class DerivedElement>
class FemElement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemElement);

  using Traits = FemElementTraits<DerivedElement>;
  using T = typename Traits::T;
  using Data = typename Traits::Data;
  using ConstitutiveModel = typename Traits::ConstitutiveModel;
  static constexpr int num_dofs = Traits::num_dofs;
  static constexpr int num_nodes = Traits::num_nodes;
  static constexpr int num_quadrature_points = Traits::num_quadrature_points;
  static constexpr bool is_linear = ConstitutiveModel::is_linear;

  /* Indices of the nodes of this element within the model. */
  const std::array<FemNodeIndex, num_nodes>& node_indices() const {
    return node_indices_;
  }

  /* Increments the node indexes of all nodes in this element by the given
   `offset`. */
  void OffsetNodeIndex(FemNodeIndex offset) {
    for (int a = 0; a < num_nodes; ++a) {
      node_indices_[a] += offset;
    }
  }

  /* Computes the per-element, state-dependent data associated with this
   `DerivedElement` given the `state` and the weights used to compute tangent
   matrices. */
  Data ComputeData(const FemState<T>& state,
                   const Vector3<T>& tangent_matrix_weights) const {
    return static_cast<const DerivedElement*>(this)->DoComputeData(
        state, tangent_matrix_weights);
  }

  /* Accumulates external forces for this element given the `data` and the
   `force_density_field`.
   @param[in] data  The FEM data to evaluate the external force.
   @param[in] plant_data  Data from the owning MultibodyPlant used to evaluate
   the external force.
   @param[in] scale  The scaling factor applied to the external force.
   @param[in, out] external_force  The vector to which the scaled external force
   will be added.
   @pre external_force != nullptr */
  void AddScaledExternalForces(
      const Data& data, const FemPlantData<T>& plant_data, const T& scale,
      EigenPtr<Vector<T, num_dofs>> external_force) const {
    DRAKE_ASSERT(external_force != nullptr);
    static_cast<const DerivedElement*>(this)->DoAddScaledExternalForces(
        data, plant_data, scale, external_force);
  }

  /* Extracts the dofs corresponding to the nodes given by `node_indices` from
   the given `state_dofs`. */
  static Vector<T, 3 * num_nodes> ExtractElementDofs(
      const std::array<FemNodeIndex, num_nodes>& node_indices,
      const VectorX<T>& state_dofs) {
    constexpr int kDim = 3;
    Vector<T, kDim * num_nodes> element_dofs;
    for (int i = 0; i < num_nodes; ++i) {
      DRAKE_ASSERT((node_indices[i] + 1) * kDim <= state_dofs.size());
      element_dofs.template segment<kDim>(i * kDim) =
          state_dofs.template segment<kDim>(node_indices[i] * kDim);
    }
    return element_dofs;
  }

  /* Extracts the dofs corresponding to the nodes in this element from the given
   `state_dofs`. */
  Vector<T, 3 * num_nodes> ExtractElementDofs(
      const VectorX<T>& state_dofs) const {
    return ExtractElementDofs(this->node_indices(), state_dofs);
  }

  /* Returns the mass of this element. */
  T mass() const { return mass_; }

 protected:
  /* Constructs a new FEM element. The constructor is made protected because
   FemElement should not be constructed directly. Use the constructor of the
   derived classes instead.
   @param[in] node_indices   The node indices of the nodes of this element
                             within the model.
   @pre Entries in node_indices are valid. */
  FemElement(const std::array<FemNodeIndex, num_nodes>& node_indices,
             ConstitutiveModel constitutive_model,
             DampingModel<T> damping_model)
      : node_indices_(node_indices),
        constitutive_model_(std::move(constitutive_model)),
        damping_model_(std::move(damping_model)) {
    for (int i = 0; i < num_nodes; ++i) {
      DRAKE_ASSERT(node_indices[i].is_valid());
    }
  }

  /* `DerivedElement` must provide an implementation for `DoComputeData()`.
   @throw std::exception if `DerivedElement` does not provide an
   implementation for `DoComputeData()`. */
  Data DoComputeData(const FemState<T>& state) const {
    ThrowIfNotImplemented(__func__);
  }

  /* `DerivedElement` must provide an implementation for
   `DoCalcInverseDynamics()` to provide the external force required to keep the
   element's state given by `data` in equilibrium. The caller guarantees that
   `external_force` is non-null and contains all zeros; the implementation in
   the derived class does not have to test for this.
   @throw std::exception if `DerivedElement` does not provide an implementation
   for `DoCalcInverseDynamics()`. */
  void DoCalcInverseDynamics(
      const Data& data, EigenPtr<Vector<T, num_dofs>> external_force) const {
    ThrowIfNotImplemented(__func__);
  }

  /* `DerivedElement` must provide an implementation for
   `DoAddScaledStiffnessMatrix()` to provide the stiffness matrix that is up to
   date given the `data`. The caller guarantees that `K` is non-null; the
   implementation in the derived class does not have to test for this.
   @throw std::exception if `DerivedElement` does not provide an implementation
   for `DoAddScaledStiffnessMatrix()`. */
  void DoAddScaledStiffnessMatrix(
      const Data& data, const T& scale,
      EigenPtr<Eigen::Matrix<T, num_dofs, num_dofs>> K) const {
    ThrowIfNotImplemented(__func__);
  }

  /* `DerivedElement` must provide an implementation for
   `DoAddScaledMassMatrix()` to provide the mass matrix that is up-to-date given
   the `data`. The caller guarantees that `M` is non-null; the implementation
   in the derived class does not have to test for this.
   @throw std::exception if `DerivedElement` does not provide an implementation
   for `DoAddScaledMassMatrix()`. */
  void DoAddScaledMassMatrix(
      const Data& data, const T& scale,
      EigenPtr<Eigen::Matrix<T, num_dofs, num_dofs>> M) const {
    ThrowIfNotImplemented(__func__);
  }

  /* `DerivedElement` must provide an implementation for
   `DoAddScaledExternalForces()` to accumulate external forces to this element
   that is up-to-date given the data and the force density evaluator. The caller
   guarantees that the output pointer is non-null; the implementation in the
   derived class does not have to test for this.
   @throw std::exception if `DerivedElement` does not provide an implementation
   for `DoAddScaledExternalForces()`. */
  void DoAddScaledExternalForces(const Data&, const FemPlantData<T>&, const T&,
                                 EigenPtr<Vector<T, num_dofs>>) const {
    ThrowIfNotImplemented(__func__);
  }

  const ConstitutiveModel& constitutive_model() const {
    return constitutive_model_;
  }

  const DampingModel<T>& damping_model() const { return damping_model_; }

  /* Sets the mass of this element. */
  void set_mass(T mass) { mass_ = mass; }

 private:
  /* Helper to throw a descriptive exception when a given function is not
   implemented. */
  void ThrowIfNotImplemented(const char* source_method) const {
    throw std::runtime_error("The DerivedElement from " +
                             NiceTypeName::Get(*this) +
                             " must provide an implementation for " +
                             std::string(source_method) + "().");
  }

  /* The node indices of this element within the model. */
  std::array<FemNodeIndex, num_nodes> node_indices_;
  /* The constitutive model that describes the stress-strain relationship
   for this element. */
  ConstitutiveModel constitutive_model_;
  DampingModel<T> damping_model_;
  /* The mass of this element. */
  T mass_{};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
