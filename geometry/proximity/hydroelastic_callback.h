#pragma once

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collision_filter_legacy.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

/** Supporting data for the shape-to-shape hydroelastic contact callback (see
 Callback below). It includes:

    - A collision filter instance.
    - The T-valued poses of _all_ geometries in the corresponding SceneGraph,
      each indexed by its corresponding geometry's GeometryIndex.
    - A vector of contact surfaces -- one instance of ContactSurface for
      every supported, unfiltered penetrating pair.

 @tparam T The computation scalar.  */
template <typename T>
struct CallbackData {
  /** Constructs the fully-specified callback data. The values are as described
   in the class documentation. The parameters are all aliased in the data and
   must remain valid at least as long as the CallbackData instance.

   @param collision_filter_in     The collision filter system. Aliased.
   @param X_WGs_in                The T-valued poses. Aliased.
   @param surfaces_in             The output results. Aliased.  */
  CallbackData(
      const CollisionFilterLegacy* collision_filter_in,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs_in,
      std::vector<ContactSurface<T>>* surfaces_in)
      : collision_filter(*collision_filter_in),
        X_WGs(*X_WGs_in),
        surfaces(*surfaces_in) {
    DRAKE_DEMAND(collision_filter_in);
    DRAKE_DEMAND(X_WGs_in);
    DRAKE_DEMAND(surfaces_in);
  }

  /** The collision filter system.  */
  const CollisionFilterLegacy& collision_filter;

  /** The T-valued poses of all geometries.  */
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs;

  /** The results of the distance query.  */
  std::vector<ContactSurface<T>>& surfaces{};
};

/** Given an object_ptr whose geometry is a box, create a coarse surface mesh for
 that box.  */
SurfaceMesh<double> MakeBoxMeshFromFcl(fcl::CollisionObjectd* object_ptr) {
  const fcl::Boxd* box_ptr = dynamic_cast<const fcl::Boxd*>(
      object_ptr->collisionGeometry().get());
  DRAKE_DEMAND(box_ptr != nullptr);
  Box box(box_ptr->side(0), box_ptr->side(1), box_ptr->side(2));
  // Edge length as long as the longest side guarantees the coarsest mesh.
  const double edge_length = box_ptr->side.maxCoeff();
  return MakeBoxSurfaceMesh<double>(box, edge_length);
}

struct SoftGeometry {
  std::unique_ptr<VolumeMesh<double>> mesh;
  std::unique_ptr<VolumeMeshFieldLinear<double, double>> p0;
};

/** Given an object_ptr whose geometry is a sphere, creates a coarse volume mesh
 field for that sphere.  */
SoftGeometry MakeSphereFromFcl(
    fcl::CollisionObjectd* object_ptr) {
  const fcl::Sphered* sphere_ptr = dynamic_cast<const fcl::Sphered*>(
      object_ptr->collisionGeometry().get());
  DRAKE_DEMAND(sphere_ptr != nullptr);
  const double r = sphere_ptr->radius;
  Sphere sphere(r);
  // Edge length is the chord length around the equator where each span is 45
  // degrees.
  SoftGeometry geometry;
  const double edge_length = 4 * r * M_PI / 16;
  geometry.mesh = std::make_unique<VolumeMesh<double>>(
      MakeSphereVolumeMesh<double>(sphere, edge_length));

  std::vector<double> p0_values;
  for (const auto& v : geometry.mesh->vertices()) {
    const Eigen::Vector3d& p_MV = v.r_MV();
    const double p_MV_len = p_MV.norm();
    p0_values.push_back(1.0 - p_MV_len / r);
  }
  geometry.p0 = std::make_unique<VolumeMeshFieldLinear<double, double>>(
      "p0", move(p0_values), geometry.mesh.get());

  return geometry;
}

/** The callback function for computing a hydroelastic contact surface between
 two arbitrary shapes.

 @param object_A_ptr    Pointer to the first object in the pair (the order has
                        no significance).
 @param object_B_ptr    Pointer to the second object in the pair (the order has
                        no significance).
 @param callback_data   Supporting data to compute the contact surface.
 @returns False; the broadphase should *not* terminate its process.
 @tparam T  The scalar type for the query.  */
template <typename T>
bool Callback(fcl::CollisionObjectd* object_A_ptr,
              fcl::CollisionObjectd* object_B_ptr,
              // NOLINTNEXTLINE
              void* callback_data) {
  auto& data = *static_cast<CallbackData<T>*>(callback_data);

  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const bool can_collide = data.collision_filter.CanCollideWith(
      encoding_a.encoding(), encoding_b.encoding());

  if (can_collide) {
    fcl::NODE_TYPE a_type = object_A_ptr->getNodeType();
    fcl::NODE_TYPE b_type = object_B_ptr->getNodeType();

    const bool valid =(a_type == fcl::GEOM_BOX && b_type == fcl::GEOM_SPHERE) ||
        (a_type == fcl::GEOM_SPHERE && b_type == fcl::GEOM_BOX);
    if (!valid) {
      throw std::logic_error(fmt::format(
          "Can't compute a contact surface between geometries {} ({}) and "
          "{} ({})",
          to_string(encoding_a.id()), GetGeometryName(*object_A_ptr),
          to_string(encoding_b.id()), GetGeometryName(*object_B_ptr)));
    }

    fcl::CollisionObjectd* object_box_ptr{};
    fcl::CollisionObjectd* object_sphere_ptr{};
    GeometryId box_id;
    GeometryId sphere_id;
    if (a_type == fcl::GEOM_BOX) {
      object_box_ptr = object_A_ptr;
      box_id = encoding_a.id();
      object_sphere_ptr = object_B_ptr;
      sphere_id = encoding_b.id();
      } else {
        object_box_ptr = object_B_ptr;
        box_id = encoding_b.id();
        object_sphere_ptr = object_A_ptr;
        sphere_id = encoding_a.id();
      }

      SurfaceMesh<double> box_mesh = MakeBoxMeshFromFcl(object_box_ptr);
      // Build the sphere.
      SoftGeometry soft_sphere = MakeSphereFromFcl(object_sphere_ptr);
      std::unique_ptr<ContactSurface<T>> surface =
          mesh_intersection::ComputeContactSurfaceFromSoftVolumeRigidSurface(
              sphere_id, *soft_sphere.p0, data.X_WGs.at(sphere_id),
              box_id, box_mesh, data.X_WGs.at(box_id));

      data.surfaces.emplace_back(std::move(*surface));
  }
  // Tell the broadphase to keep searching.
  return false;
}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
