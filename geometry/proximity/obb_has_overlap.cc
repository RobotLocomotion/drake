#include "drake/geometry/proximity/aabb.h"
#include "drake/geometry/proximity/boxes_overlap.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

bool Obb::HasOverlap(const Obb& a, const Obb& b, const RigidTransformd& X_GH) {
  // The canonical frame A of box `a` is posed in the hierarchy frame G, and
  // the canonical frame B of box `b` is posed in the hierarchy frame H.
  const RigidTransformd& X_GA = a.pose();
  const RigidTransformd& X_HB = b.pose();
  const RigidTransformd X_AB = X_GA.InvertAndCompose(X_GH * X_HB);
  return internal::BoxesOverlap(a.half_width(), b.half_width(), X_AB);
}

bool Obb::HasOverlap(const Obb& bv, const internal::Plane<double>& plane_P,
                     const math::RigidTransformd& X_PH) {
  // We want the two corners of the box that lie at the most extreme extents in
  // the plane's normal direction. Then we can determine their heights
  // -- if the interval of heights includes _zero_, the box overlaps.

  // The box's canonical frame B is posed in the hierarchy frame H.
  const RigidTransformd& X_HB = bv.pose();
  const RotationMatrixd R_PB = X_PH.rotation() * X_HB.rotation();
  // The corner of the box that will have the *greatest* height value w.r.t.
  // the plane measured from the box's frame's origin (Bo) but expressed in the
  // plane's frame.
  Vector3d p_BoCmax_P = Vector3d::Zero();
  // We want to find the vectors Bᴹᵃˣᵢ  ∈ {Bᵢ, -Bᵢ}, such that Bᴹᵃˣᵢ ⋅ n̂ₚ is
  // positive. The maximum box corner is a combination of those Bᴹᵃˣᵢ vectors.
  for (int i = 0; i < 3; ++i) {
    const Vector3d& Bi_P = R_PB.col(i);
    const Vector3d& Bi_max_P = Bi_P.dot(plane_P.normal()) > 0 ? Bi_P : -Bi_P;
    p_BoCmax_P += Bi_max_P * bv.half_width()(i);
  }

  const Vector3d& p_HoBo_H = bv.center();
  const Vector3d p_PoBo_P = X_PH * p_HoBo_H;
  // Minimum corner is merely the reflection of the maximum corner across the
  // center of the box.
  const Vector3d p_PoCmax_P = p_PoBo_P + p_BoCmax_P;
  const Vector3d p_PoCmin_P = p_PoBo_P - p_BoCmax_P;

  const double max_height = plane_P.CalcHeight(p_PoCmax_P);
  const double min_height = plane_P.CalcHeight(p_PoCmin_P);
  return min_height <= 0 && 0 <= max_height;
}

bool Obb::HasOverlap(const Obb& obb_G, const Aabb& aabb_H,
                     const RigidTransformd& X_GH) {
  /* For this analysis, aabb has local frame A and obb has local frame O.

     R_AO = R_AH * R_HG * R_GO
          = I * R_HG * R_GO                    // A is Aabb --> R_AH = R_HA = I.
          = R_HG * R_GO
     p_AO_A = R_AH * p_AO_H
            = p_AO_H                           // R_AH = R_HA = I
            = p_HO_H - p_HA_H
            = X_HG * p_GO_G - p_HA_H
            = X_HG * obb_G.center() - aabb_H.center()  */
  const RigidTransformd X_HG = X_GH.inverse();
  const RotationMatrixd R_AO = X_HG.rotation() * obb_G.pose().rotation();
  const RigidTransformd X_AO(R_AO, X_HG * obb_G.center() - aabb_H.center());
  return internal::BoxesOverlap(aabb_H.half_width(), obb_G.half_width(), X_AO);
}

bool Obb::HasOverlap(const Obb& bv, const HalfSpace&,
                     const RigidTransformd& X_CH) {
  /*
                                              Hy           Hx
                                                ╲        ╱
                        By  ╱╲       Bx          ╲      ╱
                          ╲╱  ╲    ╱              ╲    ╱
                          ╱╲   ╲  ╱                ╲  ╱
                         ╱  ╲   ╲╱                  ╲╱
                         ╲   ╲  ╱╲   bv               Ho
                          ╲   ╲╱  ╲
                           ╲   Bo  ╲
                            ╲       ╲
                             ╲      ╱
                              ╲    ╱
                               ╲  ╱        Cz
                                ╲╱         ^
                                L          ┃  Cx
              ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┺━━━>┄┄┄┄┄┄┄┄┄┄┄┄
              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  Half space
              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░

    If any point in the bounding volume has a signed distance φ that is less
    than or equal to zero, we consider the box to be overlapping the half space.
    We could simply, yet inefficiently, determine this by iterating over all
    eight vertices and evaluating the signed distance for each vertex.

    However, to provide value as a culling algorithm, we need to be cheaper. So,
    if the lowest corner (marked `L`) has a signed distance less than or equal
    to zero, the overlapping condition is met.

    The point L = Bₒ + ∑ sᵢ * dᵢ * Bᵢ, where:
      - i ∈ {x, y, z}.
      - dᵢ is the _half_ measure of the box's dimension along axis i.
      - sᵢ ∈ {1, -1}, such that sᵢBᵢ ⋅ Cz ≤ 0.

    Since, φ(p_CL) = p_CL ⋅ Cz. If p_CL is expressed in C, then the z-component
    of p_CL (p_CL_z), is equal to φ(p_CL). So, if p_CL_z ≤ 0, they overlap.
   */

  // The box's canonical frame B is posed in the hierarchy frame H.
  const RigidTransformd& X_HB = bv.pose();
  // The z-component of the position vector from box center (Bo) to the lowest
  // corner of the box (L) expressed in the half space's canonical frame C.
  const RotationMatrixd& R_CH = X_CH.rotation();
  const auto R_CB = (R_CH * X_HB.rotation()).matrix();
  double p_BL_C_z = 0.0;
  for (int i = 0; i < 3; ++i) {
    // R_CB(2, i) is Bi_C(2) --> the z-component of Bi_C.
    const double Bi_C_z = R_CB(2, i);
    const double s_i = Bi_C_z > 0 ? -1 : 1;
    p_BL_C_z += s_i * bv.half_width()(i) * Bi_C_z;
  }
  // Now we compute the z-component of the position vector from Co to L,
  // expressed in Frame C.
  //  p_CL_C = p_CB_C                   + p_BL_C
  //         = p_CH_C + p_HB_C          + p_BL_C
  //         = p_CH_C + (R_CH * p_HB_H) + p_BL_C
  // In all of these calculations, we only need the z-component. So, that means
  // we can get the z-component of p_HB_C without the full
  // R_CH * p_HB_H calculation; we can simply do Cz_H ⋅ p_HB_H.
  const Vector3d& p_HB_H = bv.center();
  const Vector3d& Cz_H = R_CH.row(2);
  const double p_HB_C_z = Cz_H.dot(p_HB_H);
  const double p_CH_C_z = X_CH.translation()(2);
  const double p_CB_C_z = p_CH_C_z + p_HB_C_z;
  const double p_CL_C_z = p_CB_C_z + p_BL_C_z;
  return p_CL_C_z <= 0;
}

}  // namespace geometry
}  // namespace drake
