#pragma once

#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Definition of a deformable body's geometry at the reference configuration.
 It includes a volume mesh and a scalar field approximating the signed distance
 field of the geometry.
 @tparam_nonsymbolic_scalar. */
template <typename T>
class ReferenceDeformableGeometry {
  /* This class is similar to geometry::internal::hydroelastic::SoftGeometry
   with a few differences:
    1. This class allows AutoDiffXd as a scalar type.
    2. This class doesn't support half space.
    3. This class doesn't provide a bounding volume hierarchy. */
 public:
  /* Constructs a deformable geometry at reference configuration with the
   provided mesh and piece-wise linear approximation of the signed distance
   field within the volume mesh. */
  ReferenceDeformableGeometry(
      std::unique_ptr<geometry::VolumeMesh<T>> mesh,
      std::unique_ptr<geometry::VolumeMeshFieldLinear<T, T>>
          signed_distance_field)
      : mesh_(std::move(mesh)),
        signed_distance_field_(std::move(signed_distance_field)) {
    DRAKE_DEMAND(mesh_.get() == &signed_distance_field_->mesh());
  }

  /* Custom copy assign and construct. */
  ReferenceDeformableGeometry<T>& operator=(
      const ReferenceDeformableGeometry<T>& s);
  ReferenceDeformableGeometry(const ReferenceDeformableGeometry<T>& s);

  ReferenceDeformableGeometry(ReferenceDeformableGeometry<T>&&) = default;
  ReferenceDeformableGeometry<T>& operator=(ReferenceDeformableGeometry<T>&&) =
      default;

  /* Returns the volume mesh representaion of the deformable geometry at
   reference configuration. */
  const geometry::VolumeMesh<T>& mesh() const { return *mesh_; }

  /* Returns a piecewise linear approximation of the signed distance field that
   is exact at mesh vertices. */
  const geometry::VolumeMeshFieldLinear<T, T>& signed_distance_field() const {
    return *signed_distance_field_;
  }

 private:
  std::unique_ptr<geometry::VolumeMesh<T>> mesh_;
  std::unique_ptr<geometry::VolumeMeshFieldLinear<T, T>> signed_distance_field_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::ReferenceDeformableGeometry);
