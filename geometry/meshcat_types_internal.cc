#include "drake/geometry/meshcat_types_internal.h"

namespace drake {
namespace geometry {
namespace internal {

GeometryData::~GeometryData() = default;

SphereGeometryData::~SphereGeometryData() = default;

CapsuleGeometryData::~CapsuleGeometryData() = default;

CylinderGeometryData::~CylinderGeometryData() = default;

BoxGeometryData::~BoxGeometryData() = default;

MeshFileGeometryData::~MeshFileGeometryData() = default;

BufferGeometryData::~BufferGeometryData() = default;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
