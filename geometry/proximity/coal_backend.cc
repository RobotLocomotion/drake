#include "drake/geometry/proximity/coal_backend.h"

#include <vector>

namespace drake {
namespace geometry {
namespace internal {

CoalBackend::FaceData CoalBackend::MakeFaces(
    const std::vector<int>& face_data) {
  FaceData triangles;
  const int size = ssize(face_data);
  int i = 0;
  while (i < size) {
    const int count = face_data[i];
    DRAKE_DEMAND(count >= 3);
    DRAKE_DEMAND(i + count < size);
    const int v0 = face_data[i + 1];
    // Fan the n-gon around its first vertex: (v0, v1, v2), (v0, v2, v3), ...
    // This preserves the winding, and therefore the outward normal.
    for (int t = 1; t + 1 < count; ++t) {
      triangles.emplace_back(v0, face_data[i + 1 + t], face_data[i + 2 + t]);
    }
    i += count + 1;
  }
  return triangles;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
