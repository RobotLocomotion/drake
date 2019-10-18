#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_nodiscard.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/mesh_field.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {

// TODO(DamrongGuoy): Consider putting shape_function_FEM into its own file.
/**
 \defgroup shape_function_FEM  Shape Functions in Finite Element Approximation

  Much of this discussion was taken from:

      O.C. Zienkiewicz, R.L. Taylor & J.Z. Zhu.
      The Finite Element Method: Its Basis and Fundamentals.
      Chapter 3. Weak Forms and Finite Element Approximation.
      Chapter 6. Shape Functions, Derivatives, and Integration.

  We approximately represent a field u over a finite domain by dividing the
  domain into "small regular shaped regions" (Zienkiewicz et al. p. 55), each
  of which define a _finite element_ domain, and a finite set of points
  shared between the finite elements define the _nodes_, where we store field
  values.  The division of the domain into elements and nodes is called a
  _finite element mesh_.  The term _element_ refers to a tetrahedron in
  VolumeMesh or a triangle in SurfaceMesh.

  On each finite element E, we have one _shape function_ Nᵢ for each
  node nᵢ of the element, where i is a local index of the node within the
  element E:

               Nᵢ : E → ℝ,

  and use Nᵢ to define the _finite element approximation_ uᵉ of Field u
  at a point p ∈ E as:

               uᵉ(p) = ∑ Nᵢ(p) * uᵢ

  where uᵢ is the value of u at the node nᵢ in the element E.  The specific
  definition of a shape function Nᵢ depends on the shape and order of
  approximation of a finite element E. For example, E could be a triangle or
  a tetrahedron, with first-order (linear) approximation, second-order
  (quadratic) approximation, etc.

  Refer to MeshFieldLinear and MeshFieldQuadratic for examples of the shape
  functions.
 */

/**@{*/

/**
 @ingroup shape_function_FEM
 %MeshFieldLinear represents a field variable defined on a finite-element
 simplicial (triangular or tetrahedral) mesh using first-order (linear)
 approximation. See @ref shape_function_FEM for basic terminology of finite
 element approximation.

 We store one field value per vertex of the mesh, and each element
 (triangle or tetrahedron) has (d+1) nodes, where d is the dimension of the
 element.

 <h3>Example. %Shape Function of a Linear Triangular Element</h3>

 A _linear triangular element_ E with three vertices v₀, v₁, v₂ has its
 three nodes n₀, n₁, n₂ are coincident with the vertices. For brevity, here we
 write vᵢ for both the label of the vertex and also the Cartesian coordinates
 of its location in a certain coordinates frame.

 <!-- TODO(DamrongGuoy): Consider simplify it. -->
 For a triangular element, it is beneficial to use a map from the
 _parent coordinate system_ (L₀, L₁, L₂) (also known as
 _barycentric_ or _area coordinates_) to the Cartesian coordinates
 p = (x,y,z) of a point on the triangle:

              p(L₀, L₁, L₂) = L₀ * v₀ + L₁ * v₁ + L₂ * v₂,
              L₀ + L₁ + L₂ = 1, Lᵢ ∈ [0,1].

 For a linear triangular element, the shape function is the same as the
 parent coordinate functions:

              Nᵢ(p) = Lᵢ(p), i = 0,1,2,

 and its finite element approximation uᵉ of Field u at a point p ∈ E is:

              uᵉ(p) = N₀(p) * u₀ + N₁(p) * u₁ + N₂(p) * u₂,

 where uᵢ is the value of u() at the node nᵢ.

 Linear tetrahedral elements are similar.

 @tparam FieldValue  a valid Eigen scalar or vector of valid Eigen scalars for
                     the field value.
 @tparam MeshType    the type of the meshes: surface mesh or volume mesh.
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

  const std::string& name() const { return name_; }
  const std::vector<FieldValue>& values() const { return values_; }
  std::vector<FieldValue>& mutable_values() { return values_; }

 private:
  // Clones MeshFieldLinear data under the assumption that the mesh
  // pointer is null.
  DRAKE_NODISCARD std::unique_ptr<MeshField<FieldValue, MeshType>>
  DoCloneWithNullMesh() const final {
    return std::make_unique<MeshFieldLinear>(*this);
  }
  std::string name_;
  // The field values are indexed in the same way as vertices, i.e.,
  // values_[i] is the field value for the mesh vertices_[i].
  std::vector<FieldValue> values_;
};

/**
 @tparam FieldValue  a valid Eigen scalar or vector of valid Eigen scalars for
                     the field value.
 @tparam T  a valid Eigen scalar for coordinates.
 */
template <typename FieldValue, typename T>
using SurfaceMeshFieldLinear = MeshFieldLinear<FieldValue, SurfaceMesh<T>>;

/**@}*/

}  // namespace geometry
}  // namespace drake

