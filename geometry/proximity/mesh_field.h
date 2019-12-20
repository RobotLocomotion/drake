#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/reset_on_copy.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {

/** %MeshField is an abstract class that represents a field variable defined
  on a mesh M. It can evaluate the field value at any location on any element
  of the mesh.

  @tparam FieldValue  a valid Eigen scalar or Vector of Eigen scalar for
                     the field value.
  @tparam MeshType   the type of the mesh: SurfaceMesh or VolumeMesh.
*/
template <class FieldValue, class MeshType>
// TODO(DamrongGuoy): Consider making the fact that MeshType is templated on
//  a scalar type explicit here for some additional error checking.
class MeshField {
 public:
  virtual ~MeshField() = default;
  /** Evaluates the field value at a vertex.
   @param v The index of the vertex.
   */
  virtual FieldValue EvaluateAtVertex(
      typename MeshType::VertexIndex e) const = 0;

  /** Evaluates the field value at a location on an element.
   @param e The index of the element.
   @param b The barycentric coordinates.
   */
  virtual FieldValue Evaluate(
      typename MeshType::ElementIndex e,
      const typename MeshType::Barycentric& b) const = 0;

  /** Evaluates the field at a point Qp on an element. If the element is a
   tetrahedron, Qp is the input point Q. If the element is a triangle, Qp is the
   projection of Q on the triangle's plane.
   @param e The index of the element.
   @param p_MQ The position of point Q expressed in frame M, in Cartesian
               coordinates. M is the frame of the mesh.
   */
  virtual FieldValue EvaluateCartesian(
      typename MeshType::ElementIndex e,
      const typename MeshType::Cartesian& p_MQ) const = 0;

  /** Copy to a new %MeshField and set the new %MeshField to use a new
   compatible mesh. %MeshField needs a mesh to operate; however, %MeshField
   does not own the mesh. In fact, several %MeshField objects can use the same
   mesh.
   */
  [[nodiscard]] std::unique_ptr<MeshField> CloneAndSetMesh(
      const MeshType* new_mesh) const {
    DRAKE_DEMAND(new_mesh != nullptr);
    DRAKE_DEMAND(new_mesh->num_vertices() == mesh_->num_vertices());
    // TODO(DamrongGuoy): Check that the `new_mesh` is equivalent to the
    //  current `mesh_M_`.
    std::unique_ptr<MeshField> new_mesh_field = CloneWithNullMesh();
    new_mesh_field->mesh_ = new_mesh;
    return new_mesh_field;
  }

  const MeshType& mesh() const { return *mesh_; }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshField)

  explicit MeshField(const MeshType* mesh): mesh_(mesh) {
    DRAKE_DEMAND(mesh_ != nullptr);
  }

  [[nodiscard]] std::unique_ptr<MeshField> CloneWithNullMesh() const {
    return DoCloneWithNullMesh();
  }
  /** Derived classes must implement this method to clone themselves given
   that the pointer to the mesh is null.
   */
  [[nodiscard]] virtual std::unique_ptr<MeshField> DoCloneWithNullMesh()
      const = 0;

 private:
  // We use `reset_on_copy` so that the default copy constructor resets
  // the pointer to null when a MeshField is copied.
  reset_on_copy<const MeshType*> mesh_;
};

/**
 @tparam FieldValue  a valid Eigen scalar or vector of valid Eigen scalars for
                     the field value.
 @tparam T  a valid Eigen scalar for coordinates.
 */
template <typename FieldValue, typename T>
using SurfaceMeshField = MeshField<FieldValue, SurfaceMesh<T>>;

extern template class MeshField<double, SurfaceMesh<double>>;
extern template class MeshField<AutoDiffXd, SurfaceMesh<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake

