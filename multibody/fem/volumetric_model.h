#pragma once

#include <array>
#include <memory>
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

/* An FEM model of 3D elastic solids in which the displacement of the material
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
                            typename Element::Quadrature, ConstitutiveModel,
                            typename Element::SubdIsoparametricElement,
                            typename Element::SubdQuadrature>,
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
                  the positions of the vertices in the reference configuration.
                  Each tetrahedral element in the input `mesh` will generate an
                  FEM element in this model.
     @param constitutive_model  The ConstitutiveModel to be used for all the
                                FEM elements created from the input mesh.
     @param density  The mass density of the new elements in the reference
                     configuration, in unit kg/m³.
     @param damping_model  The DampingModel to be used for all new elements.
     @throw std::exception if Element::Traits::num_nodes != 4. */
    void AddLinearTetrahedralElements(
        const geometry::VolumeMesh<double>& mesh,
        const ConstitutiveModel& constitutive_model, const T& density,
        const DampingModel<T>& damping_model) {
      if constexpr (Traits::num_nodes != 4 || Traits::natural_dimension != 3) {
        throw std::logic_error(
            "AddLinearTetrahedralElements() only supports linear tetrahedral "
            "elements.");
      } else {
        VectorX<T> new_reference_positions = ExtractMeshPositions(mesh);
        std::vector<Element> new_elements;
        new_elements.reserve(mesh.num_elements());

        /* Add new elements. */
        std::array<FemNodeIndex, Traits::num_nodes> element_node_indices;
        for (int e = 0; e < mesh.num_elements(); ++e) {
          for (int a = 0; a < Traits::num_nodes; ++a) {
            element_node_indices[a] = FemNodeIndex(mesh.element(e).vertex(a));
          }
          // TODO(xuchenhan-tri): The use of T over double is not well-reasoned.
          //  Consider whether T is really necessary when we support autodiff in
          //  FEM.
          const Vector<T, Traits::num_dofs> element_reference_positions =
              Element::ExtractElementDofs(element_node_indices,
                                          new_reference_positions);
          const auto& element_reference_positions_reshaped = Eigen::Map<
              const Eigen::Matrix<T, kSpatialDimension, Traits::num_nodes>>(
              element_reference_positions.data());

          Element element(element_node_indices, constitutive_model,
                          element_reference_positions_reshaped, density,
                          damping_model);
          new_elements.emplace_back(std::move(element));
        }
        reference_positions_.emplace_back(std::move(new_reference_positions));
        elements_.emplace_back(std::move(new_elements));
      }
    }

   private:
    /* Parse a tetrahedral volume mesh and return a flat vector of the vertex
     positions of the mesh. The i-th, i+1-th, i+2-th entry in the returned
     vector contains the vertex position of the i-th vertex in the mesh's frame.
    */
    VectorX<double> ExtractMeshPositions(
        const geometry::VolumeMesh<double>& mesh) {
      const int num_new_vertices = mesh.num_vertices();
      VectorX<double> q(num_new_vertices * kSpatialDimension);
      for (int v = 0; v < num_new_vertices; ++v) {
        q.template segment<kSpatialDimension>(kSpatialDimension * v) =
            mesh.vertex(v);
      }
      return q;
    }

    void DoBuild() final {
      const FemNodeIndex num_nodes_in_model(model_->num_nodes());
      const int num_new_meshes = elements_.size();
      DRAKE_DEMAND(num_new_meshes ==
                   static_cast<int>(reference_positions_.size()));
      if (num_new_meshes == 0) return;

      /* Record the node offsets for each mesh being added to the model. */
      std::vector<int> node_index_offsets(num_new_meshes, 0);
      for (int m = 1; m < num_new_meshes; ++m) {
        node_index_offsets[m] = elements_[m - 1].size();
      }

      /* Record the node offsets for each reference positions being added to the
       model and reserve space for these new dofs. */
      std::vector<int> dof_offsets(num_new_meshes, 0);
      for (int m = 1; m < num_new_meshes; ++m) {
        dof_offsets[m] = reference_positions_[m - 1].size();
      }
      const int num_new_dofs =
          dof_offsets.back() + reference_positions_.back().size();
      const int num_old_dofs = model_->reference_positions_.size();
      model_->reference_positions_.conservativeResize(num_old_dofs +
                                                      num_new_dofs);

      /* Add each mesh to the model. */
      for (int m = 0; m < num_new_meshes; ++m) {
        for (auto& e : elements_[m]) {
          e.OffsetNodeIndex(
              FemNodeIndex(num_nodes_in_model + node_index_offsets[m]));
        }
        model_->AddElements(&(elements_[m]));
        /* Clear the moved-from elements. */
        elements_[m].clear();
        model_->reference_positions_.segment(num_old_dofs + dof_offsets[m],
                                             reference_positions_[m].size()) =
            std::move(reference_positions_[m]);
      }
    }

    VolumetricModel<Element>* model_{nullptr};
    /* The i-th element contains the reference positions of the i-th mesh added.
     */
    std::vector<VectorX<T>> reference_positions_;
    /* The i-th element contains the elements of the i-th mesh added. */
    std::vector<std::vector<Element>> elements_;
  };

  /* Creates a new empty VolumetricModel. */
  explicit VolumetricModel(const Vector3<T>& tangent_matrix_weights)
      : FemModelImpl<Element>(tangent_matrix_weights) {}

  ~VolumetricModel() = default;

  // TODO(xuchenhan-tri): This can be a cache entry.
  /* Calculates the total elastic potential energy (in joules) in this
   VolumetricModel. */
  T CalcElasticEnergy(const FemState<T>& fem_state) const {
    T energy(0);
    const std::vector<typename Element::Data>& element_data =
        fem_state.template EvalElementData<typename Element::Data>(
            this->element_data_index());
    DRAKE_DEMAND(static_cast<int>(element_data.size()) == this->num_elements());
    const auto& elements = this->elements();
    for (int e = 0; e < this->num_elements(); ++e) {
      energy += elements[e].CalcElasticEnergy(element_data[e].Psi);
    }
    return energy;
  }

 private:
  std::unique_ptr<FemModel<T>> DoClone() const final {
    auto clone = std::make_unique<VolumetricModel<Element>>(
        this->tangent_matrix_weights());
    clone->reference_positions_ = reference_positions_;
    clone->SetFrom(*this);
    return clone;
  }

  // TODO(xuchenhan-tri): Consider renaming to GetReferencePositions.
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
