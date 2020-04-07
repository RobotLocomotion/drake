#pragma once

#include <array>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/mesh_field.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {

/**
 %MeshFieldLinear represents a piecewise linear scalar field f defined on a
 simplicial-element (triangular or tetrahedral) mesh; the field value
 changes linearly within each element, and the gradient is constant within
 each element. The field is continuous across adjacent elements, but its
 gradient is discontinuous from one element to the other.

 To represent a piecewise linear field f, we store one field value per vertex
 of the mesh. Each element (triangle or tetrahedron) has (d+1) vertices,
 where d is the dimension of the element. For triangle, d = 2, and for
 tetrahedron, d = 3.

 The following sections are details for interested readers.

 <h3> Barycentric coordinate </h3>

 For a linear triangle or tetrahedron element E in 3-D, we use barycentric
 coordinate:

       (b₀, b₁, b₂)     for triangle,
       (b₀, b₁, b₂, b₃) for tetrahedron,
       ∑bᵢ = 1, bᵢ ≥ 0,

 to identify a point Q that lies in the simplicial element E. The coefficient
 bᵢ is the weight of vertex Vᵢ of the element E, where the index i is a local
 index within the element E, not the global index of the entire mesh. In other
 words, vertex Vᵢ is the iᵗʰ vertex of E, not the iᵗʰ vertex among all vertices
 in the mesh. The point Q in E can be expressed as:

    Q = ∑bᵢ(Q)Vᵢ,

 where we indicate the barycentric coordinate of a point Q on an element E as
 bᵢ(Q) -- omitting E for typographical convenience.

 <h3> Field value </h3>

 At a point Q in element E, the piecewise linear field f has value:

    f(Q) = ∑bᵢ(Q)Fᵢ

 where Fᵢ is the field value at the iᵗʰ vertex of E.

 <h3> Gradient </h3>

 Consider each bᵢ as a linear scalar field on element E, the gradient of
 the piecewise linear field f on E is:

      ∇f = ∑Fᵢ∇bᵢ

 Each gradient vector ∇bᵢ is constant on E and depends on the shape of the
 triangle or tetrahedron E.

 @tparam T  a valid Eigen scalar for field values.
 @tparam MeshType    the type of the meshes: SurfaceMesh or VolumeMesh.
 */
template <class T, class MeshType>
class MeshFieldLinear final : public MeshField<T, MeshType> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshFieldLinear)

  // TODO(DamrongGuoy): Consider passing a function to evaluate the field.
  /** Constructs a MeshFieldLinear.
   @param name    The name of the field variable.
   @param values  The field value at each vertex of the mesh.
   @param mesh    The mesh M to which this MeshField refers.
   @param calculate_gradient Calculate gradient field when true, default is
                  true.
   @pre   The `mesh` is non-null, and the number of entries in `values` is the
          same as the number of vertices of the mesh.
   */
  MeshFieldLinear(std::string name, std::vector<T>&& values,
                  const MeshType* mesh, bool calculate_gradient = true)
      : MeshField<T, MeshType>(mesh),
        name_(std::move(name)), values_(std::move(values)) {
    DRAKE_DEMAND(static_cast<int>(values_.size()) ==
                 this->mesh().num_vertices());
    if (calculate_gradient) {
      CalcGradientField();
    }
  }

  T EvaluateAtVertex(typename MeshType::VertexIndex v) const final {
    return values_[v];
  }

  T Evaluate(typename MeshType::ElementIndex e,
                      const typename MeshType::Barycentric& b) const final {
    const auto& element = this->mesh().element(e);
    T value = b[0] * values_[element.vertex(0)];
    for (int i = 1; i < MeshType::kDim + 1; ++i) {
      value += b[i] * values_[element.vertex(i)];
    }
    return value;
  }

  T EvaluateCartesian(
                 typename MeshType::ElementIndex e,
                 const typename MeshType::Cartesian& p_MQ) const final {
    return Evaluate(e, this->mesh().CalcBarycentric(p_MQ, e));
  }

  /** Evaluates the gradient in the domain of the element indicated by `e`.
  The gradient is a vector in R³ expressed in frame M. For surface meshes, it
  will particularly lie parallel to the plane of the corresponding triangle.
  @throw std::runtime_error if the gradient vector was not calculated.
  */
  Vector3<T> EvaluateGradient(
      typename MeshType::ElementIndex e) const {
    if (gradients_.size() == 0) {
      throw std::runtime_error("Gradient vector was not calculated.");
    }
    return gradients_[e];
  }

  /** Transforms the gradient vectors of this field from its initial frame M
   to the new frame N.
   @warning Use this function when the reference mesh of this field changes
   its frame in the same way.
   */
  void TransformGradients(
      const math::RigidTransform<typename MeshType::ScalarType>& X_NM) {
    for (auto& grad : gradients_) {
      grad = X_NM.rotation() * grad;
    }
  }

  const std::string& name() const { return name_; }
  const std::vector<T>& values() const { return values_; }

  // TODO(#12173): Consider NaN==NaN to be true in equality tests.
  // TODO(DamrongGuoy): Change the type of parameter `field` from MeshField
  //  to MeshFieldLinear. We will need to change the callers of this function
  //  too.
  /** Checks to see whether the given MeshFieldLinear object is equal via deep
   exact comparison. The name of the objects are exempt from this comparison.
   NaNs are treated as not equal as per the IEEE standard.
   @param field The field for comparison.
   @returns `true` if the given field is equal.
   @note Requires `MeshField field` to be MeshFieldLinear.
   */
  bool Equal(const MeshField<T, MeshType>& field) const {
    if (!this->mesh().Equal(field.mesh())) return false;

    const auto* field_linear =
        dynamic_cast<const MeshFieldLinear<T, MeshType>*>(&field);
    DRAKE_DEMAND(field_linear);

    // Check field value at each vertex.
    for (typename MeshType::VertexIndex i(0); i < this->mesh().num_vertices();
         ++i) {
      if (values_.at(i) != field_linear->values_.at(i))
        return false;
    }
    // Check gradient vectors.
    if (gradients_ != field_linear->gradients_) return false;

    // All checks passed.
    return true;
  }

 private:
  // Clones MeshFieldLinear data under the assumption that the mesh
  // pointer is null.
  [[nodiscard]] std::unique_ptr<MeshField<T, MeshType>>
  DoCloneWithNullMesh() const final {
    return std::make_unique<MeshFieldLinear>(*this);
  }
  void CalcGradientField();
  Vector3<T> CalcGradientVector(
      typename MeshType::ElementIndex e) const;

  std::string name_;
  // The field values are indexed in the same way as vertices, i.e.,
  // values_[i] is the field value for the mesh vertices_[i].
  std::vector<T> values_;
  // The gradients are indexed in the same way as elements, i.e.,
  // gradients_[i] is the gradient vector on elements_[i]. The elements could
  // be tetrahedra for VolumeMesh or triangles for SurfaceMesh.
  std::vector<Vector3<T>> gradients_;
};

/**
 @tparam FieldValue  a valid Eigen scalar for field values.
 @tparam T  a valid Eigen scalar for coordinates.
 */
template <typename FieldValue, typename T>
using SurfaceMeshFieldLinear = MeshFieldLinear<FieldValue, SurfaceMesh<T>>;

extern template class MeshFieldLinear<double, SurfaceMesh<double>>;
extern template class MeshFieldLinear<AutoDiffXd, SurfaceMesh<AutoDiffXd>>;
extern template class MeshFieldLinear<double, VolumeMesh<double>>;
extern template class MeshFieldLinear<AutoDiffXd, VolumeMesh<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake

