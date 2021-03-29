#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace geometry {

template class MeshFieldLinear<double, VolumeMesh<double>>;
template class MeshFieldLinear<AutoDiffXd, VolumeMesh<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake

