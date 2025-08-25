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

bool Aabb::HasOverlap(const Aabb& bv_H, const math::RigidTransformd& X_CH) {
  /*
                                   ▲ Hy
                                   ┃
                        By         ┃        Hx
                        ▲           ━━━━━━━>
                        ┃
                    ┌───────────┐
                    │           │
           bv  ---> │     Bo    │
                    │           │
                    └───────────┘L
                        ┃
                        ▼            Cz
                                     ^
                                     ┃  Cx
              ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┺━━━>┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄
              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  Half space
              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░

    This algorithm is borrowed from Obb::HasOverlap(Obb, RigidTransformd); refer
    to it for details. The only difference is we know that the orientation of
    the AABB is the Identity matrix. The rest stays the same:

    If any point in the bounding volume has a signed distance φ that is less
    than or equal to zero, we consider the box to be overlapping the half space.
    We could simply, yet inefficiently, determine this by iterating over all
    eight vertices and evaluating the signed distance for each vertex.
  */

  // The z-component of the position vector from box center (Bo) to the lowest
  // corner of the box (L) expressed in the half space's canonical frame C.
  const RotationMatrixd& R_CH = X_CH.rotation();
  const auto R_CB = R_CH.matrix();
  double p_BL_C_z = 0.0;
  for (int i = 0; i < 3; ++i) {
    // R_CB(2, i) is Bi_C(2) --> the z-component of Bi_C.
    const double Bi_C_z = R_CB(2, i);
    const double s_i = Bi_C_z > 0 ? -1 : 1;
    p_BL_C_z += s_i * bv_H.half_width()(i) * Bi_C_z;
  }
  // Now we compute the z-component of the position vector from Co to L,
  // expressed in Frame C.
  //  p_CL_C = p_CB_C                   + p_BL_C
  //         = p_CH_C + p_HB_C          + p_BL_C
  //         = p_CH_C + (R_CH * p_HB_H) + p_BL_C
  // In all of these calculations, we only need the z-component. So, that means
  // we can get the z-component of p_HB_C without the full
  // R_CH * p_HB_H calculation; we can simply do Cz_H ⋅ p_HB_H.
  const Vector3d& p_HB_H = bv_H.center();
  const Vector3d& Cz_H = R_CH.row(2);
  const double p_HB_C_z = Cz_H.dot(p_HB_H);
  const double p_CH_C_z = X_CH.translation()(2);
  const double p_CB_C_z = p_CH_C_z + p_HB_C_z;
  const double p_CL_C_z = p_CB_C_z + p_BL_C_z;
  return p_CL_C_z <= 0;
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
