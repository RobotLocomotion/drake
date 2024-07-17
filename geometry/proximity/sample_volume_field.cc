#include "drake/geometry/proximity/sample_volume_field.h"

#include <limits>

namespace drake {
namespace geometry {
namespace internal {

double SampleVolumeFieldOrThrow(const VolumeMeshFieldLinear<double, double>& f,
                                const Vector3<double>& p) {
  // Barycentric coordinates will be positive when the point p is inside a
  // tetrahedron. When p is right on the boundary of a tetrahedron, some of the
  // barycentric coordinates will become zero. In practice however, round-off
  // errors can lead to small (order machine epsilon) negative values. To handle
  // this case robustly we use a slop.
  const double kSlop = 8 * std::numeric_limits<double>::epsilon();
  const VolumeMesh<double>& mesh = f.mesh();
  for (int e = 0; e < mesh.num_elements(); ++e) {
    const Vector4<double> b = mesh.CalcBarycentric(p, e);
    const bool p_is_inside_e = b.minCoeff() >= -kSlop;
    if (p_is_inside_e) {
      return f.Evaluate(e, b);
    }
  }
  throw std::logic_error("Point is outside the volume.");
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
