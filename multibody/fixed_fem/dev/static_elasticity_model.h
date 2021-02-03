#pragma once

#include <array>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/multibody/fixed_fem/dev/elasticity_model.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** The FEM model for static 3D elasticity problems. Implements the interface in
 FemModel. It is assumed that elements are only added to, but never deleted
 from, the model.
 @tparam Element    The type of StaticElasticityElement used in this
 %StaticElasticityModel, must be an instantiation of StaticElasticityElement. */
template <class Element>
class StaticElasticityModel : public ElasticityModel<Element> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StaticElasticityModel);

  using T = typename Element::Traits::T;
  using ConstitutiveModel = typename Element::Traits::ConstitutiveModel;

  StaticElasticityModel() = default;
  ~StaticElasticityModel() = default;

  /** Add tetrahedral StaticElasticityElements to the %StaticElasticityModel
   from a mesh. The positions of the vertices in the mesh are used as reference
   positions for the volume as well as the generalized positions for the model
   in MakeFemState(). The gravity constant for the newly added elements is given
   by ElasticityModel::gravity().
   @param mesh    The input tetrahedral mesh that describes the connectivity and
   the positions of the vertices. Each geometry::VolumeElement in the input
   `mesh` will generate a StaticElasticityElement in this
   %StaticElasticityModel.
   @param constitutive_model    The ConstitutiveModel to be used for all the
   StaticElasticityElements created from the input `mesh`.
   @throw std::exception if Element::Traits::kNumNodes != 4. */
  void AddStaticElasticityElementsFromTetMesh(
      const geometry::VolumeMesh<T>& mesh,
      const ConstitutiveModel& constitutive_model, const T& density) {
    /* Alias for more readability. */
    constexpr int kDim = Element::Traits::kSolutionDimension;
    constexpr int kNumNodes = Element::Traits::kNumNodes;
    DRAKE_THROW_UNLESS(kNumNodes == 4);

    using geometry::VolumeElementIndex;
    using geometry::VolumeVertexIndex;
    /* Record the reference positions of the input mesh. */
    const int num_new_vertices = mesh.num_vertices();
    reference_positions_.conservativeResize(
        kDim, reference_positions_.cols() + num_new_vertices);
    const NodeIndex node_index_offset = NodeIndex(this->num_nodes());
    for (VolumeVertexIndex i(0); i < num_new_vertices; ++i) {
      reference_positions_.col(i + node_index_offset) = mesh.vertex(i).r_MV();
    }

    /* Builds and adds new elements. */
    Eigen::Matrix<T, kDim, kNumNodes> element_reference_positions;
    std::array<NodeIndex, kNumNodes> element_node_indices;
    for (VolumeElementIndex i(0); i < mesh.num_elements(); ++i) {
      for (int j = 0; j < kNumNodes; ++j) {
        const int node_id = mesh.element(i).vertex(j) + node_index_offset;
        element_node_indices[j] = NodeIndex(node_id);
        element_reference_positions.col(j) = reference_positions_.col(node_id);
      }
      ElementIndex next_element_index = ElementIndex(this->num_elements());
      this->AddElement(next_element_index, element_node_indices,
                       constitutive_model, element_reference_positions, density,
                       this->gravity());
    }

    this->increment_num_nodes(num_new_vertices);
  }

 private:
  /* Implements FemModel::DoMakeFemState(). Generalized positions are
   initialized to be reference positions of the input mesh vertices. */
  FemState<Element> DoMakeFemState() const final {
    return FemState<Element>(Eigen::Map<const VectorX<T>>(
        reference_positions_.data(), reference_positions_.size()));
  }
  Matrix3X<T> reference_positions_{};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
