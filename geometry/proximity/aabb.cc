#include "drake/geometry/proximity/aabb.h"

#include "drake/geometry/proximity/boxes_overlap.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {

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
  return internal::BoxesOverlap(a_G.half_width(), b_H.half_width(), X_AB);
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
  return internal::BoxesOverlap(aabb_G.half_width(), obb_H.half_width(), X_AO);
}

bool Aabb::HasOverlap(const Aabb& bv_H, const Plane<double>& plane_P,
                      const math::RigidTransformd& X_PH) {
  const Vector3d& p_HBo = bv_H.center();
  return plane_P.BoxOverlaps(bv_H.half_width(), X_PH * p_HBo, X_PH.rotation());
}

bool Aabb::HasOverlap(const Aabb& bv_H, const HalfSpace&,
                      const math::RigidTransformd& X_CH) {
  /*
                        By  ╱╲       Bx
                          ╲╱  ╲    ╱
                          ╱╲   ╲  ╱
                         ╱  ╲   ╲╱
                         ╲   ╲  ╱╲   bv
                          ╲   ╲╱  ╲
                           ╲   Bo  ╲
                            ╲       ╲
                             ╲      ╱
                              ╲    ╱
                               ╲  ╱        Cz
                                ╲╱         ^
                                L          ┃  Cx
              ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┺━━━>┄┄┄┄┄┄┄┄┄┄┄┄┄
              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  Half space
              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░

    In the picture above, L is the point of the box that is "lowest" (in the
    opposite direction of the normal). We can project the vector from v_BoL onto
    the half space normal Cz to get the _minimum distance_ between the box
    center and the half space boundary for the box to be outside the half space.

    So, |v_BoL·Cz| is the clearance distance. The signed distance of the box
    center to the half space boundary is p_CB·Cz. The box doesn't overlap the
    half space iff p_CB·Cz > |v_BoL·Cz|.

    Given we're dotting everything with Cz, we only need the z-components of
    the quantities in question.
   */

  const RotationMatrixd& R_CH = X_CH.rotation();
  const Vector3d& p_HB = bv_H.center();
  const Vector3d& p_HB_C = R_CH * p_HB;
  const Vector3d p_CB_C = X_CH.translation() + p_HB_C;
  // Just taking the bottom row of R_CH operates on just the z-components.
  const double clearance =
      R_CH.row(2).cwiseAbs().dot(bv_H.half_width().transpose());

  return p_CB_C(2) <= clearance;
}

template <typename MeshType>
Aabb AabbMaker<MeshType>::Compute() const {
  auto itr = vertices_.begin();
  Vector3d max_bounds = internal::convert_to_double(mesh_M_.vertex(*itr));
  Vector3d min_bounds = max_bounds;
  ++itr;
  for (; itr != vertices_.end(); ++itr) {
    const Vector3d& vertex = internal::convert_to_double(mesh_M_.vertex(*itr));
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

}  // namespace geometry
}  // namespace drake
