#pragma once

#include <array>
#include <memory>
#include <utility>

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/multibody/fem/acceleration_newmark_scheme.h"
#include "drake/multibody/fem/damping_model.h"
#include "drake/multibody/fem/fem_model_impl.h"
#include "drake/multibody/fem/volumetric_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* The FEM model for dynamic 3D volumetric elasticity problems.
 @tparam Element  The type of FEM element used in this model. Must be of
 template type VolumetricElement. */
template <class Element>
class VolumetricModel : public FemModelImpl<Element> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VolumetricModel);

  using State = FemStateImpl<Element>;
  using Traits = typename Element::Traits;
  using T = typename Traits::T;
  using ConstitutiveModel = typename Traits::ConstitutiveModel;
  static constexpr int kSpatialDimension = Traits::kSpatialDimension;

  static_assert(
      std::is_same_v<
          VolumetricElement<typename Traits::IsoparametricElement,
                            typename Traits::Quadrature, ConstitutiveModel>,
          Element>,
      "The template parameter `Element` must be of type VolumetricElement.");

  /* Creates a new VolumetricModel with no elements. */
  VolumetricModel() = default;

  ~VolumetricModel() = default;

  /* Add tetrahedral FEM elements to this VolumetricModel from a mesh. The
   positions of the vertices in the mesh are used as reference positions as well
   as positions in the default FEM state.
   @param mesh  The input tetrahedral mesh that describes the connectivity and
                the positions of the vertices. Each tetrahedral element in
                the input `mesh` will generate an FEM element in this model.
   @param constitutive_model  The ConstitutiveModel to be used for all the
                              FEM elements created from the input mesh.
   @param density  The mass density of the new elements, in unit kg/m³.
   @param damping_model  The DampingModel to be used for all new elements.
   @throw std::exception if Element::Traits::num_nodes != 4. */
  void AddVolumetricElementsFromTetMesh(
      const geometry::VolumeMesh<T>& mesh,
      const ConstitutiveModel& constitutive_model, const T& density,
      const DampingModel<T>& damping_model) {
    if constexpr (Traits::num_nodes != 4) {
      throw std::logic_error(
          "AddVolumetricElementsFromTetMesh() only supports tetrahedral "
          "elements.");
    } else {
      /* Record the reference positions of the input mesh. The returned offset
       is from before the new tets are added. */
      const NodeIndex node_index_offset(this->ParseTetMesh(mesh));

      /* Add new elements. */
      std::array<NodeIndex, Traits::num_nodes> element_node_indices;
      for (int i = 0; i < mesh.num_elements(); ++i) {
        for (int j = 0; j < Traits::num_nodes; ++j) {
          /* To obtain the global node index, offset the local index of the
           nodes in the mesh (starting from 0) by the existing number of nodes
           before the current mesh is added. */
          const int node_id = mesh.element(i).vertex(j) + node_index_offset;
          element_node_indices[j] = NodeIndex(node_id);
        }
        const Vector<T, Traits::num_dofs> element_reference_positions =
            Element::ExtractElementDofs(element_node_indices,
                                        reference_positions_);
        const ElementIndex next_element_index =
            ElementIndex(this->num_elements());
        const auto& element_reference_positions_reshaped = Eigen::Map<
            const Eigen::Matrix<T, kSpatialDimension, Traits::num_nodes>>(
            element_reference_positions.data(), kSpatialDimension,
            Traits::num_nodes);
        this->AddElement(
            next_element_index, element_node_indices, constitutive_model,
            element_reference_positions_reshaped, density, damping_model);
        this->mutable_element(next_element_index)
            .set_gravity_vector(this->gravity());
      }
    }
  }

  /* Calculates the total elastic potential energy (in joules) in this
   VolumetricModel. */
  T CalcElasticEnergy(const FemStateImpl<Element>& state) const {
    T energy(0);
    for (ElementIndex i(0); i < this->num_elements(); ++i) {
      const Element& e = this->element(i);
      energy += e.CalcElasticEnergy(state);
    }
    return energy;
  }

 protected:
  /* Parse a tetrahedral volume mesh, store the positions of the vertices in
   the mesh as the reference positions for the vertices, and increment the total
   number of vertices in the model. Returns the total number of vertices
   _before_ the input `mesh` is added. */
  int ParseTetMesh(const geometry::VolumeMesh<T>& mesh) {
    /* Alias for more readability. */
    constexpr int kDim = kSpatialDimension;
    /* Record the reference positions of the input mesh. */
    const int num_new_vertices = mesh.num_vertices();
    reference_positions_.conservativeResize(reference_positions_.size() +
                                            kDim * num_new_vertices);
    /* Record the number of vertices *before* the input mesh is parsed. */
    const int num_existing_nodes = this->num_nodes();
    const NodeIndex node_index_offset = NodeIndex(num_existing_nodes);
    for (int i = 0; i < num_new_vertices; ++i) {
      reference_positions_.template segment<kDim>(
          kDim * (i + node_index_offset)) = mesh.vertex(i);
    }
    this->increment_num_nodes(num_new_vertices);
    return num_existing_nodes;
  }

  /* Returns the reference positions of all nodes in the model. */
  const VectorX<T>& reference_positions() const { return reference_positions_; }

 private:
  /* Implements FemModelImpl::DoMakeFemState(). Generalized positions are
   initialized to be reference positions of the input mesh vertices. Velocities
   and accelerations are initialized to 0. */
  FemState<T> DoMakeFemState() const final {
    const int num_dofs = reference_positions_.size();
    return FemState<T>(reference_positions_, VectorX<T>::Zero(num_dofs),
                       VectorX<T>::Zero(num_dofs));
  }

  VectorX<T> reference_positions_{};
  Vector<T, kSpatialDimension> gravity_{0, 0, -9.81};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
