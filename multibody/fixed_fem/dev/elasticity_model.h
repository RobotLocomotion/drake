#pragma once

#include <array>
#include <memory>
#include <utility>

#include "drake/multibody/fixed_fem/dev/elasticity_element.h"
#include "drake/multibody/fixed_fem/dev/fem_model.h"

namespace drake {
namespace multibody {
namespace fem {
/** The FEM model for static and dynamic 3D elasticity problems. Provides
 methods common to both StaticElasticityModel and DynamicElasticityModel.
 @tparam Element    The type of ElasticityElement used in this %ElasticityModel.
 Element must be derived from ElasticityElement. */
template <class Element>
class ElasticityModel : public FemModel<Element> {
 public:
  static_assert(
      std::is_base_of_v<
          ElasticityElement<typename Element::Traits::IsoparametricElement,
                            typename Element::Traits::Quadrature,
                            typename Element::Traits::ConstitutiveModel,
                            Element, typename Element::Traits>,
          Element>,
      "The template parameter Element must be derived from ElasticityModel.");
  static_assert(Element::Traits::kSpatialDimension == 3,
                "Only 3D elasticity is supported.");
  using T = typename Element::Traits::T;

  /** Calculates the total elastic potential energy (in joules) in this
   %ElasticityModel. */
  T CalcElasticEnergy(const FemState<Element>& state) const {
    T energy(0);
    for (ElementIndex i(0); i < this->num_elements(); ++i) {
      const Element& e = this->element(i);
      energy += e.CalcElasticEnergy(state);
    }
    return energy;
  }

  /** Sets gravity and precomputes gravity force for all elements in the model.
   The default value for gravity is [0, 0, -9.81]. */
  void SetGravity(
      const Vector<T, Element::Traits::kSpatialDimension>& gravity) {
    /* Store gravity so that all elements added after the call to this method
     get the "new" gravity constant. */
    gravity_ = gravity;
    /* Update the precomputed gravity force in elements added before the call to
     this method. */
    for (ElementIndex e(0); e < this->num_elements(); ++e) {
      this->mutable_element(e).PrecomputeGravityForce(gravity);
    }
  }

  const Vector<T, Element::Traits::kSpatialDimension> gravity() const {
    return gravity_;
  }

  /** Sets explicitly specified external forces (with units of N) on the dofs in
   the model. These explicit forces show up in the CalcResidual() and
   CalcExternalForce(). To turn off explicit external forces, pass in an empty
   vector.
   @throw std::exception if the size of `forces` is greater than zero but not
   equal to num_dofs().
   @note If the size of the model is changed after SetExplicitExternalForce() is
   called, an exception will be thrown when the model is evaluated unless
   another SetExplicitExternalForce() is called with an external force of the
   correct size. */
  void SetExplicitExternalForce(const VectorX<T>& forces) {
    if (forces.size() == 0 || forces.size() == this->num_dofs()) {
      explicit_external_forces_ = forces;
      return;
    }
    throw std::runtime_error(
        "The input force has size " + std::to_string(forces.size()) +
        ", and it doesn't match the size of the model, which has " +
        std::to_string(this->num_dofs()) + " dofs.");
  }

  /** Calculates the total external force exerted on the vertices of
   %ElasticityModel at the given `state`. The forces have units N.
   @param[in] state The FemState at which to evaluate the external force.
   @param[out] external_force The external force evaluated at `state`.
   @pre external_force != nullptr.
   @pre The size of the vector behind `external_force` must be `num_dofs()`. */
  void CalcExternalForce(const FemState<Element>& state,
                         EigenPtr<VectorX<T>> external_force) {
    DRAKE_DEMAND(external_force != nullptr);
    DRAKE_DEMAND(external_force->size() == this->num_dofs());
    DRAKE_DEMAND(state.element_cache_size() == this->num_elements());
    /* The values are accumulated in the external_force, so it is important to
     clear the old data. */
    external_force->setZero();
    /* Aliases to improve readability. */
    constexpr int kNumDofs = Element::Traits::kNumDofs;
    constexpr int kNumNodes = Element::Traits::kNumNodes;
    constexpr int kDim = Element::Traits::kSpatialDimension;
    /* Scratch space to store the contribution to the residual from each
     element. */
    Vector<T, kNumDofs> element_force;
    for (ElementIndex e(0); e < this->num_elements(); ++e) {
      element_force.setZero();
      this->element(e).AddScaledExternalForce(state, 1, &element_force);
      const std::array<NodeIndex, kNumNodes>& element_node_indices =
          this->element(e).node_indices();
      for (int i = 0; i < kNumNodes; ++i) {
        const int ei = element_node_indices[i];
        external_force->template segment<kDim>(ei * kDim) +=
            element_force.template segment<kDim>(i * kDim);
      }
    }
    /* Add in the contribution from explicitly set external force. */
    AddScaledExplicitExternalForces(1, external_force);
  }

 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElasticityModel);

  explicit ElasticityModel(
      std::unique_ptr<internal::StateUpdater<T>> state_updater)
      : FemModel<Element>(std::move(state_updater)) {}

  virtual ~ElasticityModel() = default;

  /** Parse a tetrahedral volume mesh, store the positions of the vertices in
   the mesh as the reference positions for the vertices, and increment the total
   number of vertices in the model. Returns the total number of vertices
   *before* the input `mesh` is parsed.
   @param mesh    The input tetrahedral mesh that describes the connectivity and
   the positions of the vertices. Each geometry::VolumeElement in the input
   `mesh` will generate an ElasticityElement in this ElasticityModel.
   @throw std::exception if Element::Traits::kNumNodes != 4. */
  int ParseTetMesh(const geometry::VolumeMesh<T>& mesh) {
    /* Alias for more readability. */
    constexpr int kDim = Element::Traits::kSolutionDimension;
    constexpr int kNumNodes = Element::Traits::kNumNodes;
    DRAKE_THROW_UNLESS(kNumNodes == 4);

    /* Record the reference positions of the input mesh. */
    const int num_new_vertices = mesh.num_vertices();
    reference_positions_.conservativeResize(reference_positions_.size() +
                                            kDim * num_new_vertices);
    const NodeIndex node_index_offset = NodeIndex(this->num_nodes());
    for (int i = 0; i < num_new_vertices; ++i) {
      reference_positions_.template segment<kDim>(
          kDim * (i + node_index_offset)) = mesh.vertex(i);
    }
    /* Record the number of vertices *before* the input mesh is parsed. */
    const int num_vertices = this->num_nodes();
    this->increment_num_nodes(num_new_vertices);
    return num_vertices;
  }

  const VectorX<T>& reference_positions() const { return reference_positions_; }

 private:
  /* Adds per-vertex residuals that are explicitly specified at each vertex. */
  void AddExplicitResidual(EigenPtr<VectorX<T>> residual) const final {
    /* Note that external forces enter the residual with a negative sign. */
    AddScaledExplicitExternalForces(-1, residual);
  }

  /* Adds the stored `explicit_external_forces_` scaled by `scale` to the given
   `f`. Throw an exception if their sizes don't match. */
  void AddScaledExplicitExternalForces(const T& scale,
                                       EigenPtr<VectorX<T>> f) const {
    if (explicit_external_forces_.size() == f->size()) {
      *f += scale * explicit_external_forces_;
      return;
    }
    if (explicit_external_forces_.size() != 0) {
      throw std::runtime_error(
          "The input force has size " + std::to_string(f->size()) +
          ", and it doesn't match the size of the stored explicit external "
          "force which has size " +
          std::to_string(explicit_external_forces_.size()));
    }
  }

  VectorX<T> reference_positions_{};
  Vector<T, Element::Traits::kSpatialDimension> gravity_{0, 0, -9.81};
  /* Explicit external force set by SetExplicitExternalForce. Default is empty.
   */
  VectorX<T> explicit_external_forces_{};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
