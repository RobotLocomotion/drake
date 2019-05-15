#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_nodiscard.h"

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

  DRAKE_NODISCARD
  std::unique_ptr<MeshField<FieldValue, MeshType>> Clone() const {
    return DoClone();
  }

 protected:
  /** Derived classes must implement this method to clone themselves.
   */
  DRAKE_NODISCARD
  virtual std::unique_ptr<MeshField<FieldValue, MeshType>> DoClone() const = 0;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshField)
  MeshField() = default;
};


}  // namespace geometry
}  // namespace drake

