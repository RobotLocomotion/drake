#include "drake/geometry/proximity/mesh_field.h"

namespace drake {
namespace geometry {

template class MeshField<double, SurfaceMesh<double>>;
template class MeshField<AutoDiffXd, SurfaceMesh<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake

