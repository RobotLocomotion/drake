#pragma once

#include <array>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/multibody/fem/damping_model.h"
#include "drake/multibody/fem/fem_model_impl.h"
#include "drake/multibody/fem/volumetric_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* An FEM model of 3D elastic solid in which the displacement of the material
 can be interpolated from that of the element nodes using the isoparametric
 shape function.
 @tparam Element  The type of FEM element used in this model. Must be of
 template type VolumetricElement. */
template <class Element>
class VolumetricModel : public FemModelImpl<Element> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VolumetricModel);

  using Traits = typename Element::Traits;
  using T = typename Traits::T;
  using ConstitutiveModel = typename Traits::ConstitutiveModel;
  static constexpr int kSpatialDimension = 3;

  static_assert(
      std::is_same_v<
          VolumetricElement<typename Element::IsoparametricElement,
                            typename Element::Quadrature, ConstitutiveModel>,
          Element>,
      "The template parameter `Element` must be of type VolumetricElement.");

  /* Builder that builds a VolumetricModel. */
  class VolumetricBuilder : public FemModel<T>::Builder {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VolumetricBuilder);

    /* Constructs a new builder that is associated with the given `model`. */
    explicit VolumetricBuilder(VolumetricModel<Element>* model)
        : FemModel<T>::Builder(model), model_(model) {
      DRAKE_DEMAND(model_ != nullptr);
    }

    /* Add FEM elements to this VolumetricModel from a mesh. The positions of
     the vertices in the mesh are used as reference positions as well as
     positions in the initial FEM state.
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
            "AddVolumetricElementsFromTetMesh() can only be called in builders "
            "for VolumetricModel with tetrahedral elements.");
      } else {
        /* Record the starting node index. */
        DRAKE_DEMAND(reference_positions_.size() % 3 == 0);
        const FemNodeIndex node_index_offset(reference_positions_.size() / 3);
        VectorX<T> new_reference_positions = ParseTetMesh(mesh);

        /* Add new elements. */
        std::array<FemNodeIndex, Traits::num_nodes> element_node_indices;
        for (int e = 0; e < mesh.num_elements(); ++e) {
          for (int a = 0; a < Traits::num_nodes; ++a) {
            element_node_indices[a] = FemNodeIndex(mesh.element(e).vertex(a));
          }
          const Vector<T, Traits::num_dofs> element_reference_positions =
              Element::ExtractElementDofs(element_node_indices,
                                          new_reference_positions);
          const auto& element_reference_positions_reshaped = Eigen::Map<
              const Eigen::Matrix<T, kSpatialDimension, Traits::num_nodes>>(
              element_reference_positions.data(), kSpatialDimension,
              Traits::num_nodes);

          Element element(element_node_indices, constitutive_model,
                          element_reference_positions_reshaped, density,
                          damping_model);
          /* To obtain the global node index in the builder, offset the local
           index of the nodes in the mesh (starting from 0) by the existing
           number of nodes before the current mesh is added. */
          element.OffsetNodeIndex(node_index_offset);
          elements_.emplace_back(std::move(element));
        }

        const int num_new_dofs = new_reference_positions.size();
        reference_positions_.conservativeResize(reference_positions_.size() +
                                                num_new_dofs);
        reference_positions_.tail(num_new_dofs) =
            std::move(new_reference_positions);
      }
    }

   private:
    /* Parse a tetrahedral volume mesh and return a flat vector of the vertex
     positions of the mesh. The i-th, i+1-th, i+2-th entry in the returned
     vector contains the vertex position of the i-th vertex in the mesh's frame.
    */
    VectorX<T> ParseTetMesh(const geometry::VolumeMesh<T>& mesh) {
      const int num_new_vertices = mesh.num_vertices();
      VectorX<T> q(num_new_vertices * kSpatialDimension);
      for (int a = 0; a < num_new_vertices; ++a) {
        q.template segment<kSpatialDimension>(kSpatialDimension * a) =
            mesh.vertex(a);
      }
      return q;
    }

    void DoBuild() final {
      const FemNodeIndex num_existing_nodes(model_->num_nodes());
      for (auto& e : elements_) {
        e.OffsetNodeIndex(num_existing_nodes);
      }
      model_->AddElements(&elements_);
      // Clear the moved-from elements.
      elements_.clear();
      const int num_old_dofs = model_->reference_positions_.size();
      const int num_new_dofs = reference_positions_.size();
      model_->reference_positions_.conservativeResize(num_old_dofs +
                                                      num_new_dofs);
      model_->reference_positions_.tail(num_new_dofs) =
          std::move(reference_positions_);
      // Clear the moved-from reference positions.
      reference_positions_.resize(0);
    }

    VolumetricModel<Element>* model_{nullptr};
    VectorX<T> reference_positions_;
    std::vector<Element> elements_;
  };

  /* Creates a new empty VolumetricModel. */
  VolumetricModel() = default;

  ~VolumetricModel() = default;

  // TODO(xuchenhan-tri): This can be a cache entry.
  /* Calculates the total elastic potential energy (in joules) in this
   VolumetricModel. */
  T CalcElasticEnergy(const FemState<T>& fem_state) const {
    using Data = typename Element::Data;
    T energy(0);
    const std::vector<Data>& element_data =
        fem_state.template EvalElementData<Data>(this->element_data_index());
    DRAKE_DEMAND(static_cast<int>(element_data.size()) == this->num_elements());
    for (int e(0); e < this->num_elements(); ++e) {
      energy += this->element(e).CalcElasticEnergy(element_data[e]);
    }
    return energy;
  }

 private:
  /* Implements FemModel::MakeReferencePositions(). The reference positions are
   the positions of the input mesh vertices. */
  VectorX<T> MakeReferencePositions() const final {
    return reference_positions_;
  }

  VectorX<T> reference_positions_{};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
