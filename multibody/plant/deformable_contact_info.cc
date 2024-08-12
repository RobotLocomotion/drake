#include "drake/multibody/plant/deformable_contact_info.h"

#include <utility>

using drake::geometry::GeometryId;
using drake::geometry::PolygonSurfaceMesh;

namespace drake {
namespace multibody {

template <typename T>
DeformableContactInfo<T>::DeformableContactInfo(
    GeometryId id_A, GeometryId id_B, PolygonSurfaceMesh<T> contact_mesh_W,
    SpatialForce<T> F_Ac_W)
    : id_A_(id_A),
      id_B_(id_B),
      contact_mesh_W_(std::move(contact_mesh_W)),
      F_Ac_W_(std::move(F_Ac_W)) {}

template <typename T>
DeformableContactInfo<T>::~DeformableContactInfo() = default;

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::DeformableContactInfo);
