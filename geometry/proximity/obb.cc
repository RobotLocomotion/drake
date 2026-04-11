#include "drake/geometry/proximity/obb.h"

#include <algorithm>
#include <limits>

#include "drake/geometry/proximity/aabb.h"
#include "drake/geometry/proximity/boxes_overlap.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

Obb::Obb(const RigidTransformd& X_HB, const Vector3<double>& half_width)
    : pose_(X_HB), half_width_(half_width) {
  DRAKE_DEMAND(half_width.x() >= 0.0);
  DRAKE_DEMAND(half_width.y() >= 0.0);
  DRAKE_DEMAND(half_width.z() >= 0.0);

  PadBoundary();
}

bool Obb::HasOverlap(const Obb& a, const Obb& b, const RigidTransformd& X_GH) {
  // The canonical frame A of box `a` is posed in the hierarchy frame G, and
  // the canonical frame B of box `b` is posed in the hierarchy frame H.
  const RigidTransformd& X_GA = a.pose();
  const RigidTransformd& X_HB = b.pose();
  const RigidTransformd X_AB = X_GA.InvertAndCompose(X_GH * X_HB);
  return internal::BoxesOverlap(a.half_width(), b.half_width(), X_AB);
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

bool Obb::HasOverlap(const Obb& bv_H, const Plane<double>& plane_P,
                     const math::RigidTransformd& X_PH) {
  const RotationMatrixd& R_HB = bv_H.pose().rotation();
  const Vector3d& p_HBo = bv_H.center();
  return plane_P.BoxOverlaps(bv_H.half_width(), X_PH * p_HBo,
                             X_PH.rotation() * R_HB);
}

bool Obb::HasOverlap(const Obb& bv_H, const HalfSpace&,
                     const math::RigidTransformd& X_CH) {
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
              ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┺━━━>┄┄┄┄┄┄┄┄┄┄┄┄┄
              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  Half space
              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░

    In the picture above, L is the point of the box that is "lowest" (in the
    opposite direction of the normal. We can project the vector from v_BoL onto
    the half space normal Cz to get the _minimum distance_ between the box
    center and the half space boundary for the box to be outside the half space.

    So, |v_BoL·Cz| is the clearance distance. The signed distance of the box
    center to the half space boundary is p_CB·Cz. The box doesn't overlap the
    half space iff p_CB·Cz > |v_BoL·Cz|.

    Given we're dotting everything with Cz, we only need the z-components of
    the quantities in question.
   */

  // The box's canonical frame B is posed in the hierarchy frame H.
  const RigidTransformd& X_HB = bv_H.pose();
  // The z-component of the position vector from box center (Bo) to the lowest
  // corner of the box (L) expressed in the half space's canonical frame C.
  const RotationMatrixd& R_CH = X_CH.rotation();
  const auto R_CB = (R_CH * X_HB.rotation());
  // Just taking the bottom row of R_CB operates on just the z-components.
  const double clearance =
      R_CB.row(2).cwiseAbs().dot(bv_H.half_width().transpose());

  // Now we compute the z-component of p_CB:
  //  p_CB_C = p_CH_C + p_HB_C
  //         = p_CH_C + (R_CB * p_HB_H)
  // In all of these calculations, we only need the z-component. So, that means
  // we can get the z-component of p_HB_C without the full
  // R_CH * p_HB_H calculation; we can simply do Cz_H ⋅ p_HB_H.
  const Vector3d& p_HB_H = bv_H.center();
  const Vector3d& Cz_H = R_CH.row(2);
  const double p_HB_C_z = Cz_H.dot(p_HB_H);
  const double p_CH_C_z = X_CH.translation()(2);
  const double p_CB_C_z = p_CH_C_z + p_HB_C_z;
  return p_CB_C_z <= clearance;
}

void Obb::PadBoundary() {
  const double max_position = center().cwiseAbs().maxCoeff();
  const double max_half_width = half_width_.maxCoeff();
  const double scale = std::max(max_position, max_half_width);
  const double incr =
      std::max(scale * std::numeric_limits<double>::epsilon(), kTolerance);
  half_width_ += Vector3d::Constant(incr);
}

template <class MeshType>
RotationMatrixd ObbMaker<MeshType>::CalcOrientationByPca() const {
  DRAKE_DEMAND(vertices_.size() > 0);
  // We divide by the number of vertices two times for centroid and for
  // covariance matrix. That's why we save it here.
  const double one_over_n = 1.0 / vertices_.size();

  // C is for centroid.
  Vector3d p_MC = Vector3d::Zero();
  for (int v : vertices_) {
    p_MC += internal::convert_to_double(mesh_M_.vertex(v));
  }
  p_MC *= one_over_n;

  Matrix3d covariance_M = Matrix3d::Zero();
  for (int v : vertices_) {
    const Vector3d& p_MV = internal::convert_to_double(mesh_M_.vertex(v));
    const Vector3d p_CV_M = p_MV - p_MC;
    // covariance_M is a symmetric matrix because it's a sum of the
    // 3x3 symmetric matrices V*Vᵀ of column vectors V.
    covariance_M += p_CV_M * p_CV_M.transpose();
  }
  covariance_M *= one_over_n;

  // The matrix covariance_M is selfadjoint because it is a real symmetric
  // matrix. SelfAdjointEigenSolver can always give three valid eigenvectors
  // whether the matrix is full rank or not. It can always gives three valid
  // orthonormal basis vectors even if the input point set is co-planar,
  // collinear, or just one point.
  //
  // However, an eigenvalue λ* of A = covariance_M can have algebraic
  // multiplicity m when the characteristic equation det(A-λI) = 0 has the
  // (λ*-λ)ᵐ factor, so geometrically the repeated eigenvalue λ* corresponds to
  // an m-dimensional subspace (plane for m=2, space for m=3) instead of a line,
  // and any m orthonormal basis vectors of that subspace qualify as the
  // m eigenvectors of the repeated eigenvalue λ*.
  //
  // The arbitrary choice of m eigenvectors for the eigenvalue with
  // multiplicity m can cause instability in our downstream application as
  // we explain in issue #14067.
  Eigen::SelfAdjointEigenSolver<Matrix3d> es;
  es.computeDirect(covariance_M);

  // TODO(DamrongGuoy): In case of eigenvalues with multiplicity m or near
  //  multiplicity m, pick the basis that try to optimize OBB. Right now
  //  SelfAdjointEigenSolver arbitrarily picks an orthonormal basis of the
  //  m-dimensional subspace (m-subspace for short, e.g. a plane for m=2 and
  //  the entire ℝ³ for m=3); it's not aware of our OBB application. A
  //  certain m-basis (m basis vectors of m-subspace) of the same m-subspace
  //  may give better OBBs than another one, even though both of them give valid
  //  principal components. We could use techniques like random perturbations
  //  or uniform samplings to pick a better basis of the same m-subspace.
  //  For more details, see issue #14067 ([geometry] Improve
  //  CalcOrientationByPca() for covariance matrix with repeated eigenvalues).

  // We will use only the last two eigenvectors in the next step.
  // Eigen normalized the eigenvectors already.
  const Vector3d& eigenvector2 = es.eigenvectors().col(2);
  const Vector3d& eigenvector1 = es.eigenvectors().col(1);

  // By definition, the order of the three principal components must
  // correspond to the decreasing order of the eigenvalues. We want the box
  // dimensions in Bx, By, and Bz to be in decreasing order, so the longest
  // dimension of the box is along Bx. However, Eigen returns the eigenvalues
  // and vectors in increasing order. Therefore, we are going to reverse the
  // order here.
  const Vector3d& Bx_M = eigenvector2;
  const Vector3d& By_M = eigenvector1;
  // Instead of eigenvectors().col(0), we use the cross product of the
  // other two vectors to ensure the right-handed basis.
  const Vector3d Bz_M = Bx_M.cross(By_M);

  return RotationMatrixd::MakeFromOrthonormalColumns(Bx_M, By_M, Bz_M);
}

template <class MeshType>
Obb ObbMaker<MeshType>::CalcOrientedBox(const RotationMatrixd& R_MB) const {
  // To calculate the origin Bo of frame B of the oriented box and its
  // size, we use an intermediate frame F as rotation of the mesh's frame M
  // by the given orientation R_MB of the box. In other words, the box's
  // frame B is aligned with frame F, but the origin Bo differs from Fo (Fo
  // collocates with Mo).
  //
  // This picture illustrates the concept in two dimensions:
  //
  //                                                 Bx
  //                              By          U    ⇗
  //                                ⇖       ⋰ ⋱ ⇗
  //              My                  ⇖   ⋰   ⇗ ⋱
  //              ↑                     ⇖   ⇗    ⋰
  //              ↑                   ⋰  Bo   ⋰
  //              ↑                 ⋰       ⋰
  //      Fy      ↑      Fx         ⋱     ⋰
  //        ⇖     ↑     ⇗             ⋱ ⋰
  //          ⇖   ↑   ⇗                L
  //            ⇖ ↑ ⇗
  //           Fo=Mo → → → → → → → Mx
  //
  const RotationMatrixd& R_MF = R_MB;

  // We will find the upper and lower bounding points U and L of the box as
  // measured and expressed in frame F: p_FU, p_FL. We cannot do it in frame
  // B because we do not know the origin Bo of frame B yet. It is not true in
  // general that Bo is the average positions of the vertices.
  Vector3d p_FL = Vector3d::Constant(std::numeric_limits<double>::infinity());
  Vector3d p_FU = -Vector3d::Constant(std::numeric_limits<double>::infinity());
  const RotationMatrixd R_FM = R_MF.inverse();
  for (int v : vertices_) {
    // Since frame F is a rotation of frame M with the same origin, we can use
    // the rotation R_FM for the transform X_FM.
    const Vector3d p_FV = R_FM * internal::convert_to_double(mesh_M_.vertex(v));
    p_FL = p_FL.cwiseMin(p_FV);
    p_FU = p_FU.cwiseMax(p_FV);
  }
  // Since frame B and frame F are aligned, the half_width vector expressed
  // in frame B is the same expression as in frame F.
  const Vector3d half_width = (p_FU - p_FL) / 2.;

  const Vector3d p_FBo = (p_FU + p_FL) / 2.;
  // Since frame F is a rotation of frame M with the same origin, we can use
  // the rotation R_MF for the transform X_MF.
  const Vector3d p_MBo = R_MF * p_FBo;
  return Obb(RigidTransformd(R_MB, p_MBo), half_width);
}

template <class MeshType>
Vector3d ObbMaker<MeshType>::CalcVolumeGradient(const Obb& box) const {
  const double volume_0 = box.CalcVolume();
  // The box frame B is posed in the mesh frame M.
  const RotationMatrixd& R_MB = box.pose().rotation();

  // TODO(DamrongGuoy): Use a better estimate for the step size h. This is a
  //  very big step size for calculating gradient. However, we did not see
  //  a better optimization when we changed it. Check this number again when
  //  we improve the optimization.
  constexpr double h = 5. * M_PI / 180.;  // 5-degree step.
  constexpr double one_over_h = 1.0 / h;
  // Roll, pitch, and yaw the box slightly each time by angle h to change from
  // box frame B to frames Br, Bp, and By respectively.
  static const RotationMatrixd R_BBr(RollPitchYawd(h, 0., 0.));
  static const RotationMatrixd R_BBp(RollPitchYawd(0., h, 0.));
  static const RotationMatrixd R_BBy(RollPitchYawd(0., 0., h));
  const RotationMatrixd R_MBr = R_MB * R_BBr;
  const RotationMatrixd R_MBp = R_MB * R_BBp;
  const RotationMatrixd R_MBy = R_MB * R_BBy;

  const double volume_roll = CalcOrientedBox(R_MBr).CalcVolume();
  const double volume_pitch = CalcOrientedBox(R_MBp).CalcVolume();
  const double volume_yaw = CalcOrientedBox(R_MBy).CalcVolume();

  const double dVolume_dRoll = (volume_roll - volume_0) * one_over_h;
  const double dVolume_dPitch = (volume_pitch - volume_0) * one_over_h;
  const double dVolume_dYaw = (volume_yaw - volume_0) * one_over_h;

  return {dVolume_dRoll, dVolume_dPitch, dVolume_dYaw};
}

// TODO(DamrongGuoy): Improve optimization algorithm. The unit test
//  (ObbMakerTest, TestTruncatedBox) showed that it got stuck at a local
//  optimum. See issue #14081 for more details.

template <class MeshType>
Obb ObbMaker<MeshType>::OptimizeObbVolume(const Obb& box0) const {
  // We perform a single step of gradient descent, with a line search (not a
  // Newton iteration). We searched along a negative direction of a *fixed*
  // volume-gradient vector ∂v/∂(r,p,y) in roll-pitch-yaw space. Although the
  // gradient is fixed, the step size is adjusted iteratively up or down
  // according to the new samplings.

  const Vector3d dV_dRPY = CalcVolumeGradient(box0);

  // If the gradient doesn't have an appreciable effect, skip the work to
  // optimize it further. The threshold given is based on the idea that we'd
  // like the box to change at least 1 mm in measure for a 1 radian change
  // in orientation. Since the volume is a function of the box measure cubed,
  // any change to volume less than 1e-3^3 isn't worth processing.
  const double dV_dRPY_len = dV_dRPY.norm();
  if (dV_dRPY_len <= 1e-9) return box0;

  // The box0's frame B0 is posed in the mesh frame M.
  const RotationMatrixd& R_MB0 = box0.pose().rotation();
  const double volume0 = box0.CalcVolume();

  double volume = volume0;
  Obb box = box0;

  const double kMinVolumeImprovement = 0.001;
  // Set initial step to attempt 0.1 volume reduction.
  double increment = 0.1 * volume0 / dV_dRPY_len;
  // This threshold allows shrinking the initial step size by 1/10 for 6 times.
  const double min_increment = increment / 1000000.;
  double step = 0.;
  for (int i = 0; i < 20; ++i) {
    step -= increment;
    const Obb try_box =
        CalcOrientedBox(R_MB0 * RotationMatrixd(RollPitchYawd(step * dV_dRPY)));
    const double try_volume = try_box.CalcVolume();

    if (try_volume < volume) {
      const double improvement = (volume - try_volume) / volume;
      volume = try_volume;
      box = try_box;
      if (improvement < kMinVolumeImprovement) break;
      increment *= 1.5;  // grow slowly
      continue;
    }

    // Volume does not decrease.
    step += increment;  // back to previous best
    if (increment <= min_increment) break;
    increment /= 10.;  // shrink fast
  }

  // Make sure to return the box that has volume less than or equal to the
  // original.
  return box.CalcVolume() < volume0 ? box : box0;
}

template <class MeshType>
Obb ObbMaker<MeshType>::Compute() const {
  const RotationMatrixd R_MB = CalcOrientationByPca();
  const Obb box = CalcOrientedBox(R_MB);
  return OptimizeObbVolume(box);
}

template class ObbMaker<PolygonSurfaceMesh<double>>;
template class ObbMaker<TriangleSurfaceMesh<double>>;
template class ObbMaker<VolumeMesh<double>>;

// TODO(SeanCurtis-TRI): Remove support for building a Bvh on an AutoDiff-valued
//  mesh after we've cleaned up the scalar types in hydroelastics. Specifically,
//  this is here to support the unit tests in mesh_intersection_test.cc. Also
//  the calls to internal::convert_to_double should be removed.
template class ObbMaker<PolygonSurfaceMesh<drake::AutoDiffXd>>;
template class ObbMaker<TriangleSurfaceMesh<drake::AutoDiffXd>>;
template class ObbMaker<VolumeMesh<drake::AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake
