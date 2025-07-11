#include "drake/geometry/internal_geometry.h"

#include <atomic>
#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/geometry/make_mesh_for_deformable.h"
#include "drake/geometry/proximity/calc_obb.h"

namespace drake {
namespace geometry {
namespace internal {

using math::RigidTransform;

InternalGeometry::InternalGeometry(SourceId source_id,
                                   std::unique_ptr<Shape> shape,
                                   FrameId frame_id, GeometryId geometry_id,
                                   std::string name,
                                   RigidTransform<double> X_FG)
    : shape_spec_(std::move(shape)),
      id_(geometry_id),
      name_(std::move(name)),
      source_id_(source_id),
      frame_id_(frame_id),
      X_FG_(std::move(X_FG)) {}

InternalGeometry::InternalGeometry(SourceId source_id,
                                   std::unique_ptr<Shape> shape,
                                   FrameId frame_id, GeometryId geometry_id,
                                   std::string name,
                                   RigidTransform<double> X_FG,
                                   double resolution_hint)
    : shape_spec_(std::move(shape)),
      id_(geometry_id),
      name_(std::move(name)),
      source_id_(source_id),
      frame_id_(frame_id),
      X_FG_(std::move(X_FG)) {
  // The function creates the mesh in frame G.
  reference_mesh_ = MakeMeshForDeformable(*shape_spec_, resolution_hint);
}

bool InternalGeometry::has_role(Role role) const {
  switch (role) {
    case Role::kProximity:
      return has_proximity_role();
    case Role::kIllustration:
      return has_illustration_role();
    case Role::kPerception:
      return has_perception_role();
    case Role::kUnassigned:
      return !(has_proximity_role() || has_perception_role() ||
               has_illustration_role());
  }
  DRAKE_UNREACHABLE();
}

const GeometryProperties* InternalGeometry::properties(Role role) const {
  switch (role) {
    case Role::kUnassigned:
      return nullptr;
    case Role::kProximity:
      return proximity_properties();
      break;
    case Role::kIllustration:
      return illustration_properties();
      break;
    case Role::kPerception:
      return perception_properties();
      break;
  }
  DRAKE_UNREACHABLE();
}

const std::optional<Obb>& InternalGeometry::GetObb() const {
  // TODO(jwnimmer-tri) Once we drop support for Jammy (i.e., once we can use
  // GCC >= 12 as our minimum), then we should respell these atomics to use the
  // C++20 syntax and remove the warning suppressions here and below. (We need
  // the warning suppression because newer Clang complains.)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::shared_ptr<std::optional<Obb>> check = std::atomic_load(&obb_);
#pragma GCC diagnostic pop
  if (check == nullptr) {
    // Note: This approach means that multiple threads *may* redundantly compute
    // the OBB; but only the first one will set the OBB.
    auto new_obb = std::make_shared<std::optional<Obb>>(CalcObb(*shape_spec_));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    std::atomic_compare_exchange_strong(&obb_, &check, new_obb);
#pragma GCC diagnostic pop
  }
  return *obb_;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
