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

namespace drake {
namespace geometry {

/**
 %MeshFieldLinear represents a field variable defined on a simplicial
 (triangular or tetrahedral) mesh using first-order (linear) approximation.

 We store one field value per vertex of the mesh, and each element
 (triangle or tetrahedron) has (d+1) nodes, where d is the dimension of the
 element. For triangle, d = 2, and for tetrahedron, d = 3.

 <h3> Barycentric coordinates </h3>

 For a linear triangle or tetrahedron in 3-D, we use barycentric coordinates
 bᵢ, where bᵢ is the weight of the iᵗʰ vertex, and their summation is 1.
 The barycentric coordinates B is:

        B := (b₀, b₁, b₂)       for triangle,
          := (b₀, b₁, b₂, b₃)   for tetrahedron,
       ∑bᵢ = 1, bᵢ ≥ 0.

 The corresponding Cartesian coordinates r_M of the barycentric coordinates B
 is expressed in the frame of the mesh M as:

      r_M := ∑bᵢ*r_MVᵢ,

 where r_MVᵢ is the position of vertex Vᵢ expressed in M's frame.

 <h3> Field Approximation </h3>

 At the point r_M corresponding to the barycentric coordinates B, the weighted
 sum of the field values uᵢ's at vertex Vᵢ's is the piecewise linear
 approximation uᵉ of the field u on the element E:

        uᵉ = b₀ * u₀ + b₁ * u₁ + b₂ * u₂             for triangle E,
           = b₀ * u₀ + b₁ * u₁ + b₂ * u₂ + b₃ * u₃   for tetrahedron E.

 <h3> Gradient </h3>

 Consider each barycentric coordinate bᵢ as a linear scalar field on an
 element E. The gradient ∇bᵢ is constant on E. The weighted sum of ∇bᵢ's is
 the piecewise constant approximation ∇uᵉ on E of the gradient field ∇u:

       ∇uᵉ = u₀ ∇b₀ + u₁ ∇b₁ + u₂ ∇b₂           for triangle E,
           = u₀ ∇b₀ + u₁ ∇b₁ + u₂ ∇b₂ + u₃ ∇b₃  for tetrahedron E.

 Note that ∇bᵢ is constant on E and depends on the shape of the triangle or
 tetrahedron E.

 @tparam FieldValue  a valid Eigen scalar, or a Vector of Eigen scalar.
 @tparam MeshType    the type of the meshes: SurfaceMesh or VolumeMesh.
 */
template <class FieldValue, class MeshType>
class MeshFieldLinear final : public MeshField<FieldValue, MeshType> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshFieldLinear)

  // TODO(DamrongGuoy): Consider passing a function to evaluate the field.
  /** Constructs a MeshFieldLinear.
   @param name    The name of the field variable.
   @param values  The field value at each vertex of the mesh.
   @param mesh    The mesh M to which this MeshField refers.
   @pre   The `mesh` is non-null, and the number of entries in `values` is the
          same as the number of vertices of the mesh.
   */
  MeshFieldLinear(std::string name, std::vector<FieldValue>&& values,
                  const MeshType* mesh)
      : MeshField<FieldValue, MeshType>(mesh),
        name_(std::move(name)), values_(std::move(values)) {
    DRAKE_DEMAND(static_cast<int>(values_.size()) ==
                 this->mesh().num_vertices());
    CalcGradientField();
  }

  FieldValue EvaluateAtVertex(typename MeshType::VertexIndex v) const final {
    return values_[v];
  }

  FieldValue Evaluate(typename MeshType::ElementIndex e,
                      const typename MeshType::Barycentric& b) const final {
    const auto& element = this->mesh().element(e);
    FieldValue value = b[0] * values_[element.vertex(0)];
    for (int i = 1; i < MeshType::kDim + 1; ++i) {
      value += b[i] * values_[element.vertex(i)];
    }
    return value;
  }

  FieldValue EvaluateCartesian(
                 typename MeshType::ElementIndex e,
                 const typename MeshType::Cartesian& p_MQ) const final {
    return Evaluate(e, this->mesh().CalcBarycentric(p_MQ, e));
  }

  Vector3<FieldValue> EvaluateGradient(
      typename MeshType::ElementIndex e,
      const typename MeshType::Barycentric&) const final {
    return gradients_[e];
  }

  const std::string& name() const { return name_; }
  const std::vector<FieldValue>& values() const { return values_; }
  std::vector<FieldValue>& mutable_values() { return values_; }

  /**
   Calculates gradient vectors on all elements. This function is called by the
   constructor already. However, if the mesh or the field values change, you
   might want to call this function again. For example, the frame of the mesh
   changes (SurfaceMesh::TransformVertices()), or the field values change
   through mutable_values().
   */
  void CalcGradientField();

  // TODO(#12173): Consider NaN==NaN to be true in equality tests.
  /** Checks to see whether the given MeshFieldLinear object is equal via deep
   exact comparison. The name of the objects are exempt from this comparison.
   NaNs are treated as not equal as per the IEEE standard.
   @param field The field for comparison.
   @returns `true` if the given field is equal.
   @note Requires `MeshField field` to be MeshFieldLinear.
   */
  bool Equal(const MeshField<FieldValue, MeshType>& field) const {
    if (!this->mesh().Equal(field.mesh())) return false;

    const auto* field_linear =
        dynamic_cast<const MeshFieldLinear<FieldValue, MeshType>*>(&field);
    DRAKE_DEMAND(field_linear);

    // Check field value at each vertex.
    for (typename MeshType::VertexIndex i(0); i < this->mesh().num_vertices();
         ++i) {
      if (values_.at(i) != field_linear->values_.at(i))
        return false;
    }
    for (typename MeshType::ElementIndex i(0); i < this->mesh().num_elements();
         ++i) {
      if (gradients_.at(i) != field_linear->gradients_.at(i)) return false;
    }
    // All checks passed.
    return true;
  }

 private:
  // Clones MeshFieldLinear data under the assumption that the mesh
  // pointer is null.
  [[nodiscard]] std::unique_ptr<MeshField<FieldValue, MeshType>>
  DoCloneWithNullMesh() const final {
    return std::make_unique<MeshFieldLinear>(*this);
  }
  Vector3<FieldValue> CalcGradientVector(
      typename MeshType::ElementIndex e) const;

  std::string name_;
  // The field values are indexed in the same way as vertices, i.e.,
  // values_[i] is the field value for the mesh vertices_[i].
  std::vector<FieldValue> values_;
  // The gradients are indexed in the same way as elements, i.e.,
  // gradients_[i] is the gradient vector on elements_[i]. The elements could
  // be tetrahedra for VolumeMesh or triangles for SurfaceMesh.
  std::vector<Vector3<FieldValue>> gradients_;
};

template <class FieldValue, class MeshType>
void MeshFieldLinear<FieldValue, MeshType>::CalcGradientField() {
  gradients_.clear();
  gradients_.reserve(this->mesh().num_elements());
  for (typename MeshType::ElementIndex e(0); e < this->mesh().num_elements();
       ++e) {
    gradients_.push_back(CalcGradientVector(e));
  }
}

template <class FieldValue, class MeshType>
Vector3<FieldValue> MeshFieldLinear<FieldValue, MeshType>::CalcGradientVector(
    typename MeshType::ElementIndex e) const {
  std::array<FieldValue, MeshType::kVertexPerElement> u;
  for (int i = 0; i < MeshType::kVertexPerElement; ++i) {
    u[i] = values_[this->mesh().element(e).vertex(i)];
  }
  return this->mesh().CalcGradientVectorOfLinearField(u, e);
}

/**
 @tparam FieldValue  a valid Eigen scalar or vector of valid Eigen scalars for
                     the field value.
 @tparam T  a valid Eigen scalar for coordinates.
 */
template <typename FieldValue, typename T>
using SurfaceMeshFieldLinear = MeshFieldLinear<FieldValue, SurfaceMesh<T>>;

}  // namespace geometry
}  // namespace drake

