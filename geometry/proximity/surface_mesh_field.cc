#include "drake/geometry/proximity/surface_mesh_field.h"

namespace drake {
namespace geometry {

template class MeshFieldLinear<double, SurfaceMesh<double>>;
template class MeshFieldLinear<AutoDiffXd, SurfaceMesh<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake

