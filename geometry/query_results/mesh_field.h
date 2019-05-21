#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_nodiscard.h"
#include "drake/geometry/query_results/surface_mesh.h"

namespace drake {
namespace geometry {

/** %MeshField is an abstract class that represents a field variable defined
  on a mesh. It can evaluate the field value at any location on any element
  of the mesh.

  @tparam FieldValue  a valid Eigen scalar or vector of valid Eigen scalars for
                     the field value.
  @tparam MeshType   the type of the mesh: surface mesh or volume mesh.
*/
template <class FieldValue, class MeshType>
// TODO(DamrongGuoy): Consider making the fact that MeshType is templated on
//  a scalar type explicit here for some additional error checking.
class MeshField {
 public:
  virtual ~MeshField() = default;
  /** Evaluates the field value at a location on an element.
    @param e The index of the element.
    @param b The barycentric coordinates.
   */
  virtual FieldValue Evaluate(
      const typename MeshType::ElementIndex e,
      const typename MeshType::Barycentric& b) const = 0;

  /** Copy to a new %MeshField and set the new %MeshField to use a new
    compatible mesh. %MeshField needs a mesh to operate; however, %MeshField
    does not own the mesh. In fact, several %MeshField can use the same mesh.
   */
  DRAKE_NODISCARD std::unique_ptr<MeshField> CloneWithMesh(
      MeshType* new_mesh) const {
    return DoCloneWithMesh(new_mesh);
  }

 protected:
  /** Derived classes must implement this method to clone themselves and set
    to the new mesh. It should check that the new mesh is compatible with the
    new field.
   */
  DRAKE_NODISCARD virtual std::unique_ptr<MeshField> DoCloneWithMesh(
      MeshType* new_mesh) const = 0;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshField)
  MeshField() = default;
};

/**
 @tparam FieldValue  a valid Eigen scalar or vector of valid Eigen scalars for
                     the field value.
 @tparam T  a valid Eigen scalar for coordinates.
 */
template <typename FieldValue, typename T>
using SurfaceMeshField = MeshField<FieldValue, SurfaceMesh<T>>;

}  // namespace geometry
}  // namespace drake

