#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include <fcl/fcl.h>

#include "drake/common/drake_export.h"
#include "drake/geometry/proximity/collision_filter.h"
#include "drake/geometry/proximity/mujoco_ccd_mesh_data.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* The catalog of geometries that opted into the "mujoco_multipoint" point
 contact algorithm (see DefaultProximityProperties::point_contact_algorithm):
 for each such Convex or Mesh geometry, the precomputed collision data of its
 convex hull. */
using MujocoCcdGeometries =
    std::unordered_map<GeometryId, std::shared_ptr<const MujocoCcdMeshData>>;

namespace penetration_as_point_pair DRAKE_NO_EXPORT {

/* Supporting data for the detecting collision between geometries and reporting
 them as a pair of points (see PenetrationAsPointPair). It includes:

    - A collision filter instance. Aliased.
    - An fcl collision request. Aliased.
    - The poses. Aliased.
    - A vector of point pairs -- one instance of PenetrationAsPointPair for
      every supported, unfiltered penetrating pair. Aliased. */
template <typename T>
struct CallbackData {
  CallbackData(
      const CollisionFilter* collision_filter_in,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs_in,
      std::vector<PenetrationAsPointPair<T>>* point_pairs_in)
      : collision_filter(*collision_filter_in),
        X_WGs(*X_WGs_in),
        point_pairs(*point_pairs_in) {
    DRAKE_DEMAND(collision_filter_in != nullptr);
    DRAKE_DEMAND(X_WGs_in != nullptr);
    DRAKE_DEMAND(point_pairs_in != nullptr);
    request.num_max_contacts = 1;
    request.enable_contact = true;
    // NOTE: As of 5/1/2018 the GJK implementation of Libccd appears to be
    // superior to FCL's "independent" implementation. Furthermore, libccd
    // appears to behave badly if its gjk tolerance is much tighter than
    // 2e-12. Until this changes, we explicitly specify these parameters rather
    // than relying on FCL's defaults.
    request.gjk_tolerance = 2e-12;
    request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
  }

  /* The collision filter system.  */
  const CollisionFilter& collision_filter;

  /* The parameters for the fcl object-object collision function.  */
  fcl::CollisionRequestd request;

  /** The pose of each geometry in the scene. */
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs;

  /* The results of the collision query.  */
  std::vector<PenetrationAsPointPair<T>>& point_pairs;

  /* When non-null, geometry pairs where *both* ids appear in this catalog are
   evaluated with MuJoCo's multi-point convex collision detection (producing
   up to four PenetrationAsPointPair results per geometry pair) instead of
   the single-point fcl query. Only consulted for T = double. Aliased. */
  const MujocoCcdGeometries* mujoco_ccd_geometries{nullptr};

  /* Reusable scratch memory for the MuJoCo query. */
  mutable std::vector<double> mujoco_ccd_scratch;
};

/* Callback function for FCL's collide() function for retrieving a *single*
 contact. As documented by QueryObject::ComputePointPairPenetration(), the
 result added to the output data is the same, regardless of the order of
 the two fcl objects.  */
template <typename T>
bool Callback(fcl::CollisionObjectd* fcl_object_A_ptr,
              fcl::CollisionObjectd* fcl_object_B_ptr, void* callback_data);

/* Given two objects that are candidates for a collision, returns the
 point-pair contact result. If the penetration depth turns out to be negative
 (no collision), returns nullopt. This is always the *single-point* result,
 regardless of data.mujoco_ccd_geometries. */
template <typename T>
std::optional<PenetrationAsPointPair<T>> MaybeMakePointPair(
    fcl::CollisionObjectd* fcl_object_A_ptr,
    fcl::CollisionObjectd* fcl_object_B_ptr, const CallbackData<T>& data);

/* Given two objects that are candidates for a collision, appends their
 point-pair contact results (if any) to `point_pairs`. When
 data.mujoco_ccd_geometries contains both geometries (and T = double), the
 pair is evaluated with MuJoCo's multi-point convex collision detection and
 up to four point pairs are appended; otherwise this appends the single
 MaybeMakePointPair() result. Results are appended in a deterministic order
 for fixed inputs. */
template <typename T>
void MakePointPairs(fcl::CollisionObjectd* fcl_object_A_ptr,
                    fcl::CollisionObjectd* fcl_object_B_ptr,
                    const CallbackData<T>& data,
                    std::vector<PenetrationAsPointPair<T>>* point_pairs);

// clang-format off
}  // namespace penetration_as_point_pair
// clang-format on
}  // namespace internal
}  // namespace geometry
}  // namespace drake
