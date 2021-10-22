#include "drake/geometry/proximity/triangle_surface_mesh_field.h"

namespace drake {
namespace geometry {

template class MeshFieldLinear<double, TriangleSurfaceMesh<double>>;
template class MeshFieldLinear<AutoDiffXd, TriangleSurfaceMesh<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake

