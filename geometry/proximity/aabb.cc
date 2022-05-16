#include "drake/geometry/proximity/aabb.h"

#include "drake/geometry/proximity/boxes_overlap.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

bool Aabb::HasOverlap(const Aabb& a_G, const Aabb& b_H,
                      const RigidTransformd& X_GH) {
  /* For this analysis, a_G has local frame A and b_H has local frame B.

     R_GA = R_HB = I because they are Aabb. Therefore,
     R_AB = R_AG * R_GH * R_HB
          = I * R_GH * I
          = R_GH.
     p_AB_A = R_AG * p_AB_G
            = p_AB_G                           // R_AG = R_GA = I
            = p_GB_G - p_GA_G
            = X_GH * p_HB_H - p_GA_G
            = X_GH * b_H.center() - a_G.center()  */
  const RigidTransformd X_AB(X_GH.rotation(),
                             X_GH * b_H.center() - a_G.center());
  return BoxesOverlap(a_G.half_width(), b_H.half_width(), X_AB);
}

bool Aabb::HasOverlap(const Aabb& aabb_G, const Obb& obb_H,
                      const math::RigidTransformd& X_GH) {
  /* For this analysis, aabb has local frame A and obb has local frame O.

     R_AO = R_AG * R_GH * R_HO
          = I * R_GH * R_HO                    // A is Aabb --> R_AG = R_GA = I.
          = R_GH * R_HO
     p_AO_A = R_AG * p_AO_G
            = p_AO_G                           // R_AG = R_GA = I
            = p_GO_G - p_GA_G
            = X_GH * p_HO_H - p_GA_G
            = X_GH * p_HO_H - aabb_G.center()  */
  const RigidTransformd X_AO(
      X_GH.rotation() * obb_H.pose().rotation(),
      X_GH * obb_H.pose().translation() - aabb_G.center());
  return BoxesOverlap(aabb_G.half_width(), obb_H.half_width(), X_AO);
}

template <typename MeshType>
Aabb AabbMaker<MeshType>::Compute() const {
  auto itr = vertices_.begin();
  Vector3d max_bounds = convert_to_double(mesh_M_.vertex(*itr));
  Vector3d min_bounds = max_bounds;
  ++itr;
  for (; itr != vertices_.end(); ++itr) {
    const Vector3d& vertex = convert_to_double(mesh_M_.vertex(*itr));
    // Compare its extent along each of the 3 axes.
    min_bounds = min_bounds.cwiseMin(vertex);
    max_bounds = max_bounds.cwiseMax(vertex);
  }
  const Vector3d center = (min_bounds + max_bounds) / 2;
  const Vector3d half_width = max_bounds - center;
  return Aabb(center, half_width);
}

template class AabbMaker<TriangleSurfaceMesh<double>>;
template class AabbMaker<TriangleSurfaceMesh<AutoDiffXd>>;
template class AabbMaker<VolumeMesh<double>>;
template class AabbMaker<VolumeMesh<AutoDiffXd>>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
