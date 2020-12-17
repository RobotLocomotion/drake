#pragma once

#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/multibody/fem/dev/elasticity_element.h"
#include "drake/multibody/fem/dev/elasticity_element_base.h"
#include "drake/multibody/fem/dev/elasticity_element_cache_entry.h"
#include "drake/multibody/fem/dev/fem_model.h"
#include "drake/multibody/fem/dev/linear_simplex_element.h"

namespace drake {
namespace multibody {
namespace fem {
/** The FEM model for static and dynamic 3D elasticity problems. Implements the
 abstract interface of FemModel.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class ElasticityModel : public FemModel<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElasticityModel);

  ElasticityModel() = default;

  /** The number of dimensions of the elasticity problem. */
  int solution_dimension() const final { return 3; }

  /** Add ElasticityElement's to the %ElasticityModel from a mesh. The new
   ElasticityElements created will be using LinearSimplexElement and
   SimplexGaussianQuadrature.
   @param mesh The input tetrahedral mesh that describes the connectivity and
   the positions of the vertices. Each geometry::VolumeElement in the input
   `mesh` will generate an ElasticityElement in this %ElasticityModel.
   @param density The mass density of input object in the reference
   configuration with unit kg/m³.
   @param constitutive_model The ConstitutiveModel to be used for all the
   ElasticityElements created.
   @param QuadratureOrder The order of quadrature rule used for the
   %ElasticityElement's added. Must be 1, 2 or 3.
   @pre The vertices of the input `mesh` must be locally indexed, and the
   indices must be consecutive. That is, the vertices in the `mesh` must be
   indexed consecutively from 0 to mesh.num_vertices()-1.
   @throws std::runtime_error if `quadrature_order` is not 1, 2 or 3. */
  void AddElasticityElementsFromTetMesh(
      const geometry::VolumeMesh<T>& mesh, T density,
      const ConstitutiveModel<T>& constitutive_model, int quadrature_order) {
    if (!(quadrature_order == 1 || quadrature_order == 2 ||
          quadrature_order == 3)) {
      throw std::runtime_error(
          "Only linear, quadratic, and cubic quadrature rule is supported for "
          "SimplexGaussianQuadrature");
    }
    DRAKE_DEMAND(density > 0);
    // global index = local_index + node_index_offset.
    const int num_nodes_in_mesh = mesh.num_vertices();
    const int node_index_offset = this->num_nodes();
    std::set<int> mesh_node_indices;
    for (geometry::VolumeElementIndex element_index(0);
         element_index < mesh.num_elements(); ++element_index) {
      const geometry::VolumeElement& e = mesh.element(element_index);
      std::vector<NodeIndex> element_node_indices(
          geometry::VolumeMesh<T>::kVertexPerElement);
      Eigen::Matrix<T, 3, geometry::VolumeMesh<T>::kVertexPerElement>
          element_reference_positions;
      for (int local_vertex_index = 0;
           local_vertex_index < geometry::VolumeMesh<T>::kVertexPerElement;
           ++local_vertex_index) {
        const geometry::VolumeVertexIndex vid = e.vertex(local_vertex_index);
        mesh_node_indices.insert(static_cast<int>(vid));
        element_node_indices[local_vertex_index] =
            NodeIndex(static_cast<int>(vid) + node_index_offset);
        element_reference_positions.col(local_vertex_index) =
            mesh.vertex(vid).r_MV();
      }
      std::unique_ptr<ConstitutiveModel<T>> constitutive_model_clone =
          constitutive_model.Clone();
      if (quadrature_order == 1) {
        AddElasticityElement<LinearSimplexElement<T, 3>,
                             SimplexGaussianQuadrature<T, 1, 3>>(
            element_node_indices, density, std::move(constitutive_model_clone),
            element_reference_positions);
      } else if (quadrature_order == 2) {
        AddElasticityElement<LinearSimplexElement<T, 3>,
                             SimplexGaussianQuadrature<T, 2, 3>>(
            element_node_indices, density, std::move(constitutive_model_clone),
            element_reference_positions);
      } else {
        DRAKE_ASSERT(quadrature_order == 3);
        AddElasticityElement<LinearSimplexElement<T, 3>,
                             SimplexGaussianQuadrature<T, 3, 3>>(
            element_node_indices, density, std::move(constitutive_model_clone),
            element_reference_positions);
      }
    }
    this->ThrowIfNodeIndicesAreInvalid(mesh_node_indices);
    this->increment_num_nodes(num_nodes_in_mesh);
  }

  /** Calculates the total elastic potential energy, in unit J, in the
   %ElasticityModel. */
  T CalcElasticEnergy(const FemState<T>& state) const;

 protected:
  /** Creates a new FemState with no element cache. The generalized positions
   are set to the reference positions and their derivatives are set to zero. */
  std::unique_ptr<FemState<T>> DoMakeFemState() const final;

 private:
  /* Add a single new ElasticityElement to the ElasticityModel.
   @param[in] node_indices The global node indices of the nodes of the new
   ElasticityElement.
   @param[in] density The mass density of the new ElasticityElement with unit
   kg/m³.
   @param[in] constitutive_model The ConstitutiveModel to be used for the new
   ElasticityElement.
   @param[in] reference_positions The reference positions of the nodes of the
   new ElasticityElement.
   @pre node_indices.size() must be the same as
   IsoparametricElementType::num_nodes().
   @pre reference_positions.cols() must be the same as
   IsoparametricElementType::num_nodes().
   @pre QuadratureType::kNaturalDim must be equal to
   IsoparametricElementType::kNaturalDim.
   @tparam IsoparametricElementType The type of IsoparametricElement used in
   this ElasticityModel. IsoparametricElementType must be a derived class from
   IsoparametricElement.
   @tparam QuadratureType The type of Quadrature used in this ElasticityModel.
   QuadratureType must be a derived class from Quadrature. */
  template <class IsoparametricElementType, class QuadratureType>
  void AddElasticityElement(
      const std::vector<NodeIndex>& node_indices, const T& density,
      std::unique_ptr<ConstitutiveModel<T>> constitutive_model,
      const Matrix3X<T>& reference_positions) {
    static_assert(is_isoparametric_element<IsoparametricElementType>::value,
                  "IsoparametricElementType must be a derived class of "
                  "IsoparametricElement<T, NaturalDim>, where NaturalDim can "
                  "be 1, 2 or 3.");
    static_assert(is_quadrature<QuadratureType>::value,
                  "QuadratureType must be a derived class of "
                  "Quadrature<T, NaturalDim>, where NaturalDim can "
                  "be 1, 2 or 3.");
    static_assert(
        IsoparametricElementType::kNaturalDim == QuadratureType::kNaturalDim,
        "The dimension of the parent domain for IsoparametricElement and "
        "Quadrature must be the same.");
    // TODO(xuchenhan-tri): This assumes that elements are never deleted which
    // may not be true in the future (e.g. when we support fracture).
    ElementIndex element_index(this->num_elements());
    /* The number of reference positions should be the same as the number of
     nodes in the element. */
    DRAKE_DEMAND(reference_positions.cols() ==
                 static_cast<int>(node_indices.size()));
    for (int i = 0; i < static_cast<int>(node_indices.size()); ++i) {
      /* Verify mesh consistency among elements. In other words, a single node
       cannot be assigned with different reference positions in different
       elements. */
      if (reference_positions_.find(node_indices[i]) !=
          reference_positions_.end()) {
        DRAKE_DEMAND(reference_positions_[node_indices[i]] ==
                     reference_positions.col(i));
      } else {
        reference_positions_[node_indices[i]] = reference_positions.col(i);
      }
    }
    /* Create the element. */
    this->AddElement(
        std::make_unique<
            ElasticityElement<T, IsoparametricElementType, QuadratureType>>(
            element_index, node_indices, density, std::move(constitutive_model),
            reference_positions));
  }

  /* Static cast the input `element` to be of type const
   ElasticityElementBase<T>&. */
  const ElasticityElementBase<T>& get_elasticity_element_base(
      const ElementIndex& element_index) const {
    return static_cast<const ElasticityElementBase<T>&>(
        this->element(element_index));
  }

  /* 'reference_positions_' is used to verify mesh consistency. A single node
   cannot be assigned with different reference positions in different
   elements. */
  std::unordered_map<NodeIndex, Vector3<T>> reference_positions_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::ElasticityModel);
