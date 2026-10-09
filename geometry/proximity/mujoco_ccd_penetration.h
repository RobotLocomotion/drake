#pragma once

#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/mujoco_ccd_mesh_data.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Computes the penetration between two convex meshes using MuJoCo's native
 convex collision detection: GJK decides whether the two hulls penetrate, EPA
 finds the penetration direction, and — when the contact involves a face of at
 least one mesh — a multi-point contact manifold is recovered by clipping the
 two participating faces against each other. Face-face and edge-face contacts
 produce up to four contact points (MuJoCo prunes larger clipped polygons to
 the maximum-area quadrilateral); vertex-face contacts produce the single
 deepest point, matching Drake's existing single-point behavior.

 Each returned point pair reports its own depth, so contact points on a
 slightly tilted face are graded rather than sharing one depth.

 Non-penetrating geometries append nothing. Points with depth below machine
 epsilon are dropped, mirroring the osculation guard in Drake's existing
 point-pair fallback.

 @param mesh_A      Precomputed collision data of geometry A's convex hull.
 @param X_WA        Pose of geometry A in the world frame.
 @param mesh_B      Precomputed collision data of geometry B's convex hull.
 @param X_WB        Pose of geometry B in the world frame.
 @param id_A        Reported as PenetrationAsPointPair::id_A.
 @param id_B        Reported as PenetrationAsPointPair::id_B.
 @param scratch     Reusable scratch memory for MuJoCo's polytope and
                    clipping buffers; resized as needed. Passing the same
                    vector across calls avoids reallocation.
 @param point_pairs The results are appended to this vector, in a
                    deterministic order for fixed inputs.
 @returns true when MuJoCo resolved the pair: the hulls are separated (nothing
          is appended) or at least one usable contact point was appended.
          Returns false, appending nothing, when MuJoCo's GJK/EPA degenerated
          on the pair. That happens when the two hulls' vertex centroids
          coincide (GJK then stalls at its starting point and reports a
          distance of exactly zero without running EPA) and when EPA returns
          non-finite witness points under very deep penetration. The caller
          should then fall back to another algorithm rather than report no
          contact for a penetrating pair. */
bool ComputeMujocoMultipointPenetration(
    const MujocoCcdMeshData& mesh_A, const math::RigidTransformd& X_WA,
    const MujocoCcdMeshData& mesh_B, const math::RigidTransformd& X_WB,
    GeometryId id_A, GeometryId id_B, std::vector<double>* scratch,
    std::vector<PenetrationAsPointPair<double>>* point_pairs);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
