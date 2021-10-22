#include "drake/geometry/proximity/obb.h"

#include <algorithm>
#include <limits>

#include "drake/geometry/proximity/aabb.h"
#include "drake/geometry/proximity/boxes_overlap.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

bool Obb::HasOverlap(const Obb& a, const Obb& b,
                     const RigidTransformd& X_GH) {
  // The canonical frame A of box `a` is posed in the hierarchy frame G, and
  // the canonical frame B of box `b` is posed in the hierarchy frame H.
  const RigidTransformd& X_GA = a.pose();
  const RigidTransformd& X_HB = b.pose();
  const RigidTransformd X_AB = X_GA.InvertAndCompose(X_GH * X_HB);
  return BoxesOverlap(a.half_width(), b.half_width(), X_AB);
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
  const RotationMatrixd R_AO =
      X_HG.rotation() * obb_G.pose().rotation();
  const RigidTransformd X_AO(R_AO, X_HG * obb_G.center() - aabb_H.center());
  return BoxesOverlap(aabb_H.half_width(), obb_G.half_width(), X_AO);
}

bool Obb::HasOverlap(const Obb& bv, const Plane<double>& plane_P,
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

bool Obb::HasOverlap(const Obb& bv, const HalfSpace&,
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
    p_MC += convert_to_double(mesh_M_.vertex(v));
  }
  p_MC *= one_over_n;

  Matrix3d covariance_M = Matrix3d::Zero();
  for (int v : vertices_) {
    const Vector3d& p_MV = convert_to_double(mesh_M_.vertex(v));
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
    const Vector3d p_FV = R_FM * convert_to_double(mesh_M_.vertex(v));
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
  constexpr double one_over_h = 1./h;
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

  // The box0's frame B0 is posed in the mesh frame M.
  const RotationMatrixd& R_MB0 = box0.pose().rotation();
  const double volume0 = box0.CalcVolume();

  double volume = volume0;
  Obb box = box0;
  const Vector3d dV_dRPY = CalcVolumeGradient(box0);
  const double kMinVolumeImprovement = 0.001;
  // Set initial step to attempt 0.1 volume reduction.
  double increment = 0.1 * volume0 / dV_dRPY.norm();
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

template class ObbMaker<TriangleSurfaceMesh<double>>;
template class ObbMaker<VolumeMesh<double>>;

// TODO(SeanCurtis-TRI): Remove support for building a Bvh on an AutoDiff-valued
//  mesh after we've cleaned up the scalar types in hydroelastics. Specifically,
//  this is here to support the unit tests in mesh_intersection_test.cc. Also
//  the calls to convert_to_double should be removed.
template class ObbMaker<TriangleSurfaceMesh<drake::AutoDiffXd>>;
template class ObbMaker<VolumeMesh<drake::AutoDiffXd>>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake

