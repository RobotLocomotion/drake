#include "drake/geometry/proximity/mesh_field_linear.h"

namespace drake {
namespace geometry {

template class MeshFieldLinear<double, SurfaceMesh<double>>;
template class MeshFieldLinear<AutoDiffXd, SurfaceMesh<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake

