#include "drake/solvers/rotation_constraint.h"
#include "drake/solvers/rotation_constraint_internal.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <vector>

#include "drake/math/cross_product.h"

namespace drake {
namespace solvers {

MatrixDecisionVariable<3, 3> NewRotationMatrixVars(MathematicalProgram* prog,
                                                   const std::string& name) {
  MatrixDecisionVariable<3, 3> R = prog->NewContinuousVariables<3, 3>(name);

  // Forall i,j, -1 <= R(i,j) <=1.
  prog->AddBoundingBoxConstraint(-1, 1, R);

  // -1 <= trace(R) <= 3.
  // Proof sketch:
  //   orthonormal => |lambda_i|=1.
  //   R is real => eigenvalues either real or appear in complex conj pairs.
  //   Case 1: All real (lambda_i \in {-1,1}).
  //     det(R)=lambda_1*lambda_2*lambda_3=1 => lambda_1=lambda_2, lambda_3=1.
  //   Case 2: Two imaginary, pick conj(lambda_1) = lambda_2.
  //    => lambda_1*lambda_2 = 1.  =>  lambda_3 = 1.
  //    and also => lambda_1 + lambda_2 = 2*Re(lambda_1) \in [-2,2].
  prog->AddLinearConstraint(Eigen::RowVector3d::Ones(), -1, 3, R.diagonal());
  return R;
}

void AddBoundingBoxConstraintsImpliedByRollPitchYawLimits(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R,
    RollPitchYawLimits limits) {
  // Based on the RPY to Rotation Matrix conversion:
  // [ cp*cy, cy*sp*sr - cr*sy, sr*sy + cr*cy*sp]
  // [ cp*sy, cr*cy + sp*sr*sy, cr*sp*sy - cy*sr]
  // [   -sp,            cp*sr,            cp*cr]
  // where cz = cos(z) and sz = sin(z), and using
  //  kRoll_NegPI_2_to_PI_2 = 1 << 1,   // => cos(r)>=0
  //  kRoll_0_to_PI = 1 << 2,           // => sin(r)>=0
  //  kPitch_NegPI_2_to_PI_2 = 1 << 3,  // => cos(p)>=0
  //  kPitch_0_to_PI = 1 << 4,          // => sin(p)>=0
  //  kYaw_NegPI_2_to_PI_2 = 1 << 5,    // => cos(y)>=0
  //  kYaw_0_to_PI = 1 << 6,            // => sin(y)>=0

  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kYaw_NegPI_2_to_PI_2))
    prog->AddBoundingBoxConstraint(0, 1, R(0, 0));

  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kYaw_0_to_PI))
    prog->AddBoundingBoxConstraint(0, 1, R(1, 0));

  if (limits & kPitch_0_to_PI) prog->AddBoundingBoxConstraint(-1, 0, R(2, 0));

  if ((limits & kRoll_NegPI_2_to_PI_2) && (limits & kYaw_NegPI_2_to_PI_2) &&
      (limits & kPitch_0_to_PI) && (limits & kRoll_0_to_PI) &&
      (limits & kYaw_0_to_PI))
    prog->AddBoundingBoxConstraint(0, 1, R(1, 1));

  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kRoll_0_to_PI))
    prog->AddBoundingBoxConstraint(0, 1, R(2, 1));

  if ((limits & kRoll_0_to_PI) && (limits & kYaw_0_to_PI) &&
      (limits & kRoll_NegPI_2_to_PI_2) && (limits & kYaw_NegPI_2_to_PI_2) &&
      (limits & kPitch_0_to_PI))
    prog->AddBoundingBoxConstraint(0, 1, R(0, 2));

  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kRoll_NegPI_2_to_PI_2))
    prog->AddBoundingBoxConstraint(0, 1, R(2, 2));
}

void AddBoundingBoxConstraintsImpliedByRollPitchYawLimitsToBinary(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& B,
    RollPitchYawLimits limits) {
  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kYaw_NegPI_2_to_PI_2))
    prog->AddBoundingBoxConstraint(1, 1, B(0, 0));

  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kYaw_0_to_PI))
    prog->AddBoundingBoxConstraint(1, 1, B(1, 0));

  if (limits & kPitch_0_to_PI) prog->AddBoundingBoxConstraint(0, 0, B(2, 0));

  if ((limits & kRoll_NegPI_2_to_PI_2) && (limits & kYaw_NegPI_2_to_PI_2) &&
      (limits & kPitch_0_to_PI) && (limits & kRoll_0_to_PI) &&
      (limits & kYaw_0_to_PI))
    prog->AddBoundingBoxConstraint(1, 1, B(1, 1));

  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kRoll_0_to_PI))
    prog->AddBoundingBoxConstraint(1, 1, B(2, 1));

  if ((limits & kRoll_0_to_PI) && (limits & kYaw_0_to_PI) &&
      (limits & kRoll_NegPI_2_to_PI_2) && (limits & kYaw_NegPI_2_to_PI_2) &&
      (limits & kPitch_0_to_PI))
    prog->AddBoundingBoxConstraint(1, 1, B(0, 2));

  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kRoll_NegPI_2_to_PI_2))
    prog->AddBoundingBoxConstraint(1, 1, B(2, 2));
}

void AddRotationMatrixSpectrahedralSdpConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R) {
  // TODO(russt): Clean this up using symbolic expressions!
  Eigen::Matrix4d F0 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d F11 = Eigen::Matrix4d::Zero();
  F11(0, 0) = -1;
  F11(1, 1) = 1;
  F11(2, 2) = 1;
  F11(3, 3) = -1;
  Eigen::Matrix4d F21 = Eigen::Matrix4d::Zero();
  F21(0, 2) = -1;
  F21(1, 3) = 1;
  F21(2, 0) = -1;
  F21(3, 1) = 1;
  Eigen::Matrix4d F31 = Eigen::Matrix4d::Zero();
  F31(0, 1) = 1;
  F31(1, 0) = 1;
  F31(2, 3) = 1;
  F31(3, 2) = 1;
  Eigen::Matrix4d F12 = Eigen::Matrix4d::Zero();
  F12(0, 2) = 1;
  F12(1, 3) = 1;
  F12(2, 0) = 1;
  F12(3, 1) = 1;
  Eigen::Matrix4d F22 = Eigen::Matrix4d::Zero();
  F22(0, 0) = -1;
  F22(1, 1) = -1;
  F22(2, 2) = 1;
  F22(3, 3) = 1;
  Eigen::Matrix4d F32 = Eigen::Matrix4d::Zero();
  F32(0, 3) = 1;
  F32(1, 2) = -1;
  F32(2, 1) = -1;
  F32(3, 0) = 1;
  Eigen::Matrix4d F13 = Eigen::Matrix4d::Zero();
  F13(0, 1) = 1;
  F13(1, 0) = 1;
  F13(2, 3) = -1;
  F13(3, 2) = -1;
  Eigen::Matrix4d F23 = Eigen::Matrix4d::Zero();
  F23(0, 3) = 1;
  F23(1, 2) = 1;
  F23(2, 1) = 1;
  F23(3, 0) = 1;
  Eigen::Matrix4d F33 = Eigen::Matrix4d::Zero();
  F33(0, 0) = 1;
  F33(1, 1) = -1;
  F33(2, 2) = 1;
  F33(3, 3) = -1;

  prog->AddLinearMatrixInequalityConstraint(
      {F0, F11, F21, F31, F12, F22, F32, F13, F23, F33},
      {R.col(0), R.col(1), R.col(2)});
}

namespace {

void AddOrthogonalConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorDecisionVariable<3>>& v1,
    const Eigen::Ref<const VectorDecisionVariable<3>>& v2) {
  // We do this by introducing
  //   |v1+v2|^2 = v1'v1 + 2v1'v2 + v2'v2 <= 2
  //   |v1-v2|^2 = v1'v1 - 2v1'v2 + v2'v2 <= 2
  // This is tight when v1'v1 = 1 and v2'v2 = 1.

  // TODO(russt): Consider generalizing this to |v1+alpha*v2|^2 <= 1+alpha^2,
  // for any real-valued alpha.  When |R1|<|R2|<=1 or |R2|<|R1|<=1,
  // different alphas represent different constraints.

  Eigen::Matrix<double, 5, 6> A;
  Eigen::Matrix<double, 5, 1> b;

  // |v1+v2|^2 <= 2
  // Implemented as a Lorenz cone using z = Ax+b = [ sqrt(2); v1+v2 ].
  Vector4<symbolic::Expression> z;
  z << std::sqrt(2), v1 + v2;
  prog->AddLorentzConeConstraint(z);

  // |v1-v2|^2 <= 2
  // Implemented as a Lorenz cone using z = Ax+b = [ sqrt(2); v1-v2 ].
  z.tail<3>() = v1 - v2;
  prog->AddLorentzConeConstraint(z);
}

}  // namespace

void AddRotationMatrixOrthonormalSocpConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R) {
  // All columns should be unit length (but we can only write Ri'Ri<=1),
  // implemented as a rotated Lorenz cone with z = Ax+b = [1;1;R.col(i)].
  Eigen::Matrix<double, 5, 3> A = Eigen::Matrix<double, 5, 3>::Zero();
  A.bottomRows<3>() = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 5, 1> b;
  b << 1, 1, 0, 0, 0;
  for (int i = 0; i < 3; i++) {
    prog->AddRotatedLorentzConeConstraint(A, b, R.col(i));
    prog->AddRotatedLorentzConeConstraint(A, b, R.row(i).transpose());
  }

  AddOrthogonalConstraint(prog, R.col(0), R.col(1));  // R0'*R1 = 0.
  AddOrthogonalConstraint(prog, R.col(1), R.col(2));  // R1'*R2 = 0.
  AddOrthogonalConstraint(prog, R.col(0), R.col(2));  // R0'*R2 = 0.

  // Same for the rows
  AddOrthogonalConstraint(prog, R.row(0).transpose(), R.row(1).transpose());
  AddOrthogonalConstraint(prog, R.row(1).transpose(), R.row(2).transpose());
  AddOrthogonalConstraint(prog, R.row(0).transpose(), R.row(2).transpose());
}

namespace {

// Decodes the discretization of the axes.
// For compactness, this method is referred to as phi(i) in the documentation
// below.  The implementation must give a valid number even for i<0 and
// i>num_binary_variables_per_half_axis.
double EnvelopeMinValue(int i, int num_binary_variables_per_half_axis) {
  return static_cast<double>(i) / num_binary_variables_per_half_axis;
}

// Given (an integer enumeration of) the orthant, takes a vector in the
// positive orthant into that orthant by flipping the signs of the individual
// elements.
Eigen::Vector3d FlipVector(const Eigen::Ref<const Eigen::Vector3d>& vpos,
                           int orthant) {
  DRAKE_ASSERT(vpos(0) >= 0 && vpos(1) >= 0 && vpos(2) >= 0);
  DRAKE_DEMAND(orthant >= 0 && orthant <= 7);
  Eigen::Vector3d v = vpos;
  if (orthant & (1 << 2)) v(0) = -v(0);
  if (orthant & (1 << 1)) v(1) = -v(1);
  if (orthant & 1) v(2) = -v(2);
  return v;
}

// Given (an integer enumeration of) the orthant, return a vector c with
// c(i) = a(i) if element i is positive in the indicated orthant, otherwise
// c(i) = b(i).
template <typename Derived>
Eigen::Matrix<Derived, 3, 1> PickPermutation(
    const Eigen::Matrix<Derived, 3, 1>& a,
    const Eigen::Matrix<Derived, 3, 1>& b, int orthant) {
  DRAKE_DEMAND(orthant >= 0 && orthant <= 7);
  Eigen::Matrix<Derived, 3, 1> c = a;
  if (orthant & (1 << 2)) c(0) = b(0);
  if (orthant & (1 << 1)) c(1) = b(1);
  if (orthant & 1) c(2) = b(2);
  return c;
}

// Given two coordinates, find the (positive) third coordinate on that
// intersects with the unit circle.
double Intercept(double x, double y) {
  DRAKE_ASSERT(x * x + y * y <= 1);
  return std::sqrt(1 - x * x - y * y);
}

}  // namespace

namespace internal {
// Given an axis-aligned box in the first orthant, computes and returns all the
// intersecting points between the edges of the box and the unit sphere.
// @param bmin  The vertex of the box closest to the origin.
// @param bmax  The vertex of the box farthest from the origin.
std::vector<Eigen::Vector3d> ComputeBoxEdgesAndSphereIntersection(
    const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax) {
  // Assumes the positive orthant (and bmax > bmin).
  DRAKE_ASSERT(bmin(0) >= 0 && bmin(1) >= 0 && bmin(2) >= 0);
  DRAKE_ASSERT(bmax(0) > bmin(0) && bmax(1) > bmin(1) && bmax(2) > bmin(2));

  // Assumes the unit circle intersects the box.
  DRAKE_ASSERT(bmin.lpNorm<2>() <= 1);
  DRAKE_ASSERT(bmax.lpNorm<2>() >= 1);

  std::vector<Eigen::Vector3d> intersections;

  if (bmin.lpNorm<2>() == 1) {
    // Then only the min corner intersects.
    intersections.push_back(bmin);
    return intersections;
  }

  if (bmax.lpNorm<2>() == 1) {
    // Then only the max corner intersects.
    intersections.push_back(bmax);
    return intersections;
  }

  // The box has at most 12 edges, each edge can intersect with the unit sphere
  // at most once, since the box is in the first orthant.
  intersections.reserve(12);

  // 1. Loop through each vertex of the box, add it to intersections if
  // the vertex is on the sphere.
  for (int i = 0; i < 8; ++i) {
    Eigen::Vector3d vertex{};
    for (int axis = 0; axis < 3; ++axis) {
      vertex(axis) = i & (1 << axis) ? bmin(axis) : bmax(axis);
    }
    if (vertex.norm() == 1) {
      intersections.push_back(vertex);
    }
  }

  // 2. Loop through each edge, find the intersection between each edge and the
  // unit sphere, if one exists.
  for (int axis = 0; axis < 3; ++axis) {
    // axis = 0 means edges along x axis;
    // axis = 1 means edges along y axis;
    // axis = 2 means edges along z axis;
    int fixed_axis1 = (axis + 1) % 3;
    int fixed_axis2 = (axis + 2) % 3;
    // 4 edges along each axis;

    // First finds the two end points on the edge.
    Eigen::Vector3d pt_closer, pt_farther;
    pt_closer(axis) = bmin(axis);
    pt_farther(axis) = bmax(axis);
    std::array<double, 2> fixed_axis1_val = {
        {bmin(fixed_axis1), bmax(fixed_axis1)}};
    std::array<double, 2> fixed_axis2_val = {
        {bmin(fixed_axis2), bmax(fixed_axis2)}};
    for (double val1 : fixed_axis1_val) {
      pt_closer(fixed_axis1) = val1;
      pt_farther(fixed_axis1) = pt_closer(fixed_axis1);
      for (double val2 : fixed_axis2_val) {
        pt_closer(fixed_axis2) = val2;
        pt_farther(fixed_axis2) = pt_closer(fixed_axis2);

        // Determines if there is an intersecting point between the edge and the
        // sphere. If the intersecting point is not the vertex of the box, then
        // push this intersecting point to intersections directly.
        if (pt_closer.norm() < 1 && pt_farther.norm() > 1) {
          Eigen::Vector3d pt_intersect{};
          pt_intersect(fixed_axis1) = pt_closer(fixed_axis1);
          pt_intersect(fixed_axis2) = pt_closer(fixed_axis2);
          pt_intersect(axis) =
              Intercept(pt_intersect(fixed_axis1), pt_intersect(fixed_axis2));
          intersections.push_back(pt_intersect);
        }
      }
    }
  }
  return intersections;
}

/*
 * For the intersection region between the surface of the unit sphere, and the
 * interior of a box aligned with the axes, use a half space relaxation for
 * the intersection region as nᵀ * v >= d
 * @param[in] pts. The vertices containing the intersecting points between edges
 * of the box and the surface of the unit sphere.
 * @param[out] n. The unit length normal vector of the halfspace, pointing
 * outward.
 * @param[out] d. The intercept of the halfspace.
 */
void ComputeHalfSpaceRelaxationForBoxSphereIntersection(
    const std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d* n, double* d) {
  DRAKE_DEMAND(pts.size() >= 3);
  // We first prove that for a given normal vector n, and ANY unit
  // length vector v within the intersection region between the
  // surface of the unit sphere and the interior of the axis-aligned
  // box, the minimal of nᵀ * v, always occurs at one of the vertex of
  // the intersection region, if the box and the vector n are in the
  // same orthant. Namely min nᵀ * v = min(nᵀ * pts.col(i))
  // To see this, for any vector v in the intersection region, suppose
  // it is on an arc, aligned with one axis. Without loss of
  // generality we assume the aligned axis is x axis, namely
  // v(0) = t, box_min(0) <= t <= box_max(0)
  // and v(1)² + v(2)² = 1 - t², with the bounds
  // box_min(1) <= v(1) <= box_max(1)
  // box_min(2) <= v(2) <= box_max(2)
  // And the inner product nᵀ * v =
  // n(0) * t + s * (n(1) * cos(α) + n(2) * sin(α))
  // where we define s = sqrt(1 - t²)
  // Using the property of trigonometric function, we know that
  // the minimal of (n(1) * cos(α) + n(2) * sin(α)) is obtained at
  // the boundary of α. Thus we know that the minimal of nᵀ * v is
  // always obtained at one of the vertex pts.col(i).

  // To find the tightest bound d satisfying nᵀ * v >= d for all
  // vector v in the intersection region, we use the fact that for
  // a given normal vector n, the minimal of nᵀ * v is always obtained
  // at one of the vertices pts.col(i), and formulate the following
  // SOCP to find the normal vector n
  // max d
  // s.t d <= nᵀ * pts.col(i)
  //     nᵀ * n <= 1
  MathematicalProgram prog_normal;
  auto n_var = prog_normal.NewContinuousVariables<3>();
  auto d_var = prog_normal.NewContinuousVariables<1>();
  prog_normal.AddLinearCost(Vector1d(-1), d_var);
  for (const auto& pt : pts) {
    prog_normal.AddLinearConstraint(Eigen::Vector4d(pt(0), pt(1), pt(2), -1), 0,
                                    std::numeric_limits<double>::infinity(),
                                    {n_var, d_var});
  }
  // A_lorentz * n + b_lorentz = [1; n]
  Eigen::Matrix<double, 4, 3> A_lorentz{};
  A_lorentz << Eigen::RowVector3d::Zero(), Eigen::Matrix3d::Identity();
  Eigen::Vector4d b_lorentz(1, 0, 0, 0);
  prog_normal.AddLorentzConeConstraint(A_lorentz, b_lorentz, n_var);
  prog_normal.Solve();
  *n = prog_normal.GetSolution(n_var);
  *d = prog_normal.GetSolution(d_var(0));

  DRAKE_DEMAND((*n)(0) > 0 && (*n)(1) > 0 && (*n)(2) > 0);
  DRAKE_DEMAND(*d > 0 && *d < 1);
}
}  // namespace internal

namespace {

void AddMcCormickVectorConstraints(
    MathematicalProgram* prog, const VectorDecisionVariable<3>& v,
    const std::vector<MatrixDecisionVariable<3, 1>>& cpos,
    const std::vector<MatrixDecisionVariable<3, 1>>& cneg,
    const VectorDecisionVariable<3>& v1, const VectorDecisionVariable<3>& v2) {
  const int N = cpos.size();  // number of discretization points.

  // Iterate through regions.
  Eigen::Vector3d box_min, box_max;
  for (int xi = 0; xi < N; xi++) {
    box_min(0) = EnvelopeMinValue(xi, N);
    box_max(0) = EnvelopeMinValue(xi + 1, N);
    for (int yi = 0; yi < N; yi++) {
      box_min(1) = EnvelopeMinValue(yi, N);
      box_max(1) = EnvelopeMinValue(yi + 1, N);
      for (int zi = 0; zi < N; zi++) {
        box_min(2) = EnvelopeMinValue(zi, N);
        box_max(2) = EnvelopeMinValue(zi + 1, N);

        // If min corner is inside the unit circle...
        if (box_min.lpNorm<2>() < 1.0) {
          VectorDecisionVariable<3> this_cpos, this_cneg;
          this_cpos << cpos[xi](0), cpos[yi](1), cpos[zi](2);
          this_cneg << cneg[xi](0), cneg[yi](1), cneg[zi](2);

          if (box_max.lpNorm<2>() <= 1.0) {
            // This box is not allowed, e.g. c[xi]+c[yi]+c[zi] <= 2

            // Should never happen on the outer boxes.
            DRAKE_DEMAND((xi < (N - 1)) && (yi < (N - 1)) && (zi < (N - 1)));

            for (int o = 0; o < 8; o++) {  // iterate over orthants
              prog->AddLinearConstraint(
                  Eigen::RowVector3d(1, 1, 1), 0.0, 2.0,
                  PickPermutation(this_cpos, this_cneg, o));
            }
          } else {
            // Find the intercepts of the unit sphere with the box, then find
            // the tightest linear constraint of the form:
            //    d <= n'*v
            // that puts v inside (but as close as possible to) the unit circle.
            auto pts = internal::ComputeBoxEdgesAndSphereIntersection(box_min,
                                                                      box_max);
            DRAKE_DEMAND(pts.size() >= 3);

            double d(0);
            Eigen::Vector3d normal{};
            internal::ComputeHalfSpaceRelaxationForBoxSphereIntersection(
                pts, &normal, &d);

            // theta is the maximal angle between v and normal, where v is an
            // intersecting point between the box and the sphere.
            double cos_theta = d;
            const double theta = std::acos(cos_theta);

            Eigen::Matrix<double, 1, 6> a;
            Eigen::Matrix<double, 3, 9> A_cross;

            Eigen::RowVector3d orthant_normal;
            VectorDecisionVariable<3> orthant_c;
            for (int o = 0; o < 8; o++) {  // iterate over orthants
              orthant_normal = FlipVector(normal, o).transpose();
              orthant_c = PickPermutation(this_cpos, this_cneg, o);

              // Minimum vector norm constraint: normal.dot(v) >= d,
              // Since we only apply this when the box is active, since 0<=d<=1,
              // and allowing normal.dot(v) to take values at least [-1,1]
              // otherwise, the complete constraint is
              //   normal'*x >= d - 6+2*c[xi](0)+2*c[yi](1)+2*c[zi](2)
              // or, in words:
              //   if c[xi](0) = 1 and c[yi](1) == 1 and c[zi](2) == 1, then
              //     normal'*x >= d,
              //   otherwise
              //     normal'*x >= -1.
              a << orthant_normal, -2, -2, -2;
              prog->AddLinearConstraint(a, d - 6, 1, {v, orthant_c});

              // Max vector norm constraint: -1 <= normal'*x <= 1.
              // No need to restrict to this orthant, but also no need to apply
              // the same constraint twice (would be the same for opposite
              // orthants), so skip all of the -x orthants.
              if (o % 2 == 0)
                prog->AddLinearConstraint(orthant_normal, -1, 1, v);

              // Dot-product constraint: ideally v.dot(v1) = v.dot(v2) = 0.
              // The cone of (unit) vectors within theta of the normal vector
              // defines a band of admissible vectors v1 and v2 which are
              // orthogonal to v.  They must satisfy the constraint:
              //    -sin(theta) <= normal.dot(vi) <= sin(theta)
              // Proof sketch:
              //   v is within theta of normal.
              //   => vi must be within theta of a vector orthogonal to
              //      the normal.
              //   => vi must be pi/2 +/- theta from the normal.
              //   => |normal||vi| cos(pi/2 + theta) <= normal.dot(vi) <=
              //                |normal||vi| cos(pi/2 - theta).
              // Since normal and vi are both unit length,
              //     -sin(theta) <= normal.dot(vi) <= sin(theta).
              // To activate this only when this box is active, we use
              //   -sin(theta)-6+2*c[xi](0)+2*c[yi](1)+2*c[zi](2) <=
              //   normal.dot(vi)
              //     normal.dot(vi) <=
              //     sin(theta)+6-2*c[xi](0)-2*c[yi](1)-2*c[zi](2).
              // Note: (An alternative tighter, but SOCP constraint)
              //   v, v1, v2 forms an orthornormal basis. So n'*v is the
              //   projection of n in the v direction, same for n'*v1, n'*v2.
              //   Thus
              //     (n'*v)^2 + (n'*v1)^2 + (n'*v2)^2 = n'*n
              //   which translates to "The norm of a vector is equal to the
              //   sum of squares of the vector projected onto each axes of an
              //   orthornormal basis".
              //   This equation is the same as
              //     (n'*v1)^2 + (n'*v2)^2 <= sin(theta)^2
              //   So actually instead of imposing
              //     -sin(theta)<=n'*vi <=sin(theta),
              //   we can impose a tighter Lorentz cone constraint
              //     [|sin(theta)|, n'*v1, n'*v2] is in the Lorentz cone.
              prog->AddLinearConstraint(a, -sin(theta) - 6, 1, {v1, orthant_c});
              prog->AddLinearConstraint(a, -sin(theta) - 6, 1, {v2, orthant_c});

              a.tail<3>() << 2, 2, 2;
              prog->AddLinearConstraint(a, -1, sin(theta) + 6, {v1, orthant_c});
              prog->AddLinearConstraint(a, -1, sin(theta) + 6, {v2, orthant_c});

              // Cross-product constraint: ideally v2 = v.cross(v1).
              // Since v is within theta of normal, we will prove that
              // |v2 - normal.cross(v1)| <= 2 * sin(theta / 2)
              // Notice that (v2 - normal.cross(v1))ᵀ * (v2 - normal.cross(v1))
              // = v2ᵀ * v2 + (normal.cross(v1))ᵀ * (normal.cross(v1)) -
              //      2 * v2ᵀ * (normal.cross(v1))
              // <= 1 + 1 - 2 * cos(theta)
              // = (2 * sin(theta / 2))²
              // Thus we get |v2 - normal.cross(v1)| <= 2 * sin(theta / 2)
              // Here we consider to use an elementwise linear constraint
              // -2*sin(theta / 2) <=  v2 - normal.cross(v1) <= 2*sin(theta / 2)
              // Since 0<=theta<=pi/2, this should be enough to rule out the
              // det(R)=-1 case (the shortest projection of a line across the
              // circle onto a single axis has length 2sqrt(3)/3 > 1.15), and
              // can be significantly tighter.

              // To activate this only when the box is active, the complete
              // constraints are
              //  -2*sin(theta/2)-6+2(cxi+cyi+czi) <= v2-normal.cross(v1)
              //    v2-normal.cross(v1) <= 2*sin(theta/2)+6-2(cxi+cyi+czi)
              // Note: Again this constraint could be tighter as a Lorenz cone
              // constraint of the form:
              //   |v2 - normal.cross(v1)| <= 2*sin(theta/2).
              A_cross << Eigen::Matrix3d::Identity(),
                  -math::VectorToSkewSymmetric(orthant_normal),
                  Eigen::Matrix3d::Constant(-2);
              prog->AddLinearConstraint(
                  A_cross, Eigen::Vector3d::Constant(-2 * sin(theta / 2) - 6),
                  Eigen::Vector3d::Constant(2), {v2, v1, orthant_c});

              A_cross.rightCols<3>() = Eigen::Matrix3d::Constant(2.0);
              prog->AddLinearConstraint(
                  A_cross, Eigen::Vector3d::Constant(-2),
                  Eigen::Vector3d::Constant(2 * sin(theta / 2) + 6),
                  {v2, v1, orthant_c});
            }
          }
        }
      }
    }
  }
}
}  // namespace

void AddRotationMatrixMcCormickEnvelopeMilpConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R,
    int num_binary_vars_per_half_axis, RollPitchYawLimits limits) {
  DRAKE_DEMAND(num_binary_vars_per_half_axis >= 1);

  // Use a simple lambda to make the constraints more readable below.
  // Note that
  //  forall k>=0, 0<=phi(k), and
  //  forall k<=num_binary_vars_per_half_axis, phi(k)<=1.
  auto phi = [&](int k) -> double {
    return EnvelopeMinValue(k, num_binary_vars_per_half_axis);
  };

  // Creates binary decision variables which discretize each axis.
  //   Bpos[k](i,j) = 1 <=> R(i,j) >= phi(k)
  //   Bneg[k](i,j) = 1 <=> R(i,j) <= -phi(k)
  //
  // For convenience, we introduce additional (continuous) variables to
  // represent the individual sections of the real line
  //   Cpos[k](i,j) = Bpos[k](i,j) if k=N-1, otherwise
  //   Cpos[k](i,j) = Bpos[k](i,j) - Bpos[k+1](i,j)
  // This is useful only because the *number of decision variables* that we
  // pass into the constraints changes for the k=N-1 case.  Otherwise we
  // could do a simple substitution everywhere.
  // TODO(russt): Use symbolic constraints and remove these decision variables!
  std::vector<MatrixDecisionVariable<3, 3>> Bpos, Bneg;
  std::vector<MatrixDecisionVariable<3, 3>> Cpos, Cneg;
  for (int k = 0; k < num_binary_vars_per_half_axis; k++) {
    Bpos.push_back(prog->NewBinaryVariables<3, 3>("BRpos" + std::to_string(k)));
    Bneg.push_back(prog->NewBinaryVariables<3, 3>("BRneg" + std::to_string(k)));
    Cpos.push_back(
        prog->NewContinuousVariables<3, 3>("CRpos" + std::to_string(k)));
    Cneg.push_back(
        prog->NewContinuousVariables<3, 3>("CRneg" + std::to_string(k)));
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < num_binary_vars_per_half_axis; k++) {
        // R(i,j) > phi(k) => Bpos[k](i,j) = 1
        // R(i,j) < phi(k) => Bpos[k](i,j) = 0
        // R(i,j) = phi(k) => Bpos[k](i,j) = 0 or 1
        // -2 + 2*Bpos[k](i,j) <= R(i,j)-phi(k) <= Bpos[k](i,j)

        // Tight on the lower bound:
        prog->AddLinearConstraint(
            Eigen::RowVector2d(1, -2), -2 + phi(k), phi(k),
            VectorDecisionVariable<2>{R(i, j), Bpos[k](i, j)});
        // Tight on the upper bound:
        prog->AddLinearConstraint(
            Eigen::RowVector2d(1, -1), -2 + phi(k), phi(k),
            VectorDecisionVariable<2>{R(i, j), Bpos[k](i, j)});

        // -R(i,j) >= phi(k) => Bneg[k](i,j) = 1
        // -R(i,j) <= phi(k) => Bneg[k](i,j) = 0
        // -R(i,j) = phi(k) => Bneg[k](i,j) = 0 or 1
        // -Bneg[k](i,j) <= R(i,j)+phi(k) <= 2-2*Bneg[k](i,j)

        // Tight on the lower bound:
        prog->AddLinearConstraint(R(i, j) + Bneg[k](i, j), -phi(k), 2- phi(k));

        // Tight on the lower bound:
        prog->AddLinearConstraint(R(i, j) + 2 * Bneg[k](i, j), -phi(k), 2 - phi(k));

        if (k == num_binary_vars_per_half_axis - 1) {
          //   Cpos[k](i,j) = Bpos[k](i,j)
          prog->AddLinearEqualityConstraint(Cpos[k](i, j) - Bpos[k](i, j), 0);

          //   Cneg[k](i,j) = Bneg[k](i,j)
          prog->AddLinearEqualityConstraint(Cneg[k](i, j) - Bneg[k](i, j), 0);
        } else {
          //   Cpos[k](i,j) = Bpos[k](i,j) - Bpos[k+1](i,j)
          prog->AddLinearConstraint(Cpos[k](i, j) == Bpos[k](i, j) - Bpos[k + 1](i, j));
          //   Cneg[k](i,j) = Bneg[k](i,j) - Bneg[k+1](i,j)
          prog->AddLinearConstraint(Cneg[k](i, j) == Bneg[k](i, j) - Bneg[k + 1](i, j));
        }
      }
      // Bpos[0](i,j) + Bneg[0](i,j) = 1.  (have to pick a side).
      prog->AddLinearConstraint(Bpos[0](i, j) + Bneg[0](i, j) == 1);

      // for debugging: constrain to positive orthant.
      //      prog->AddBoundingBoxConstraint(1,1,{Bpos[0].block<1,1>(i,j)});
    }
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 1; k < num_binary_vars_per_half_axis; ++k) {
        // Bpos[k-1](i,j) = 0 => R(i,j) <= phi(k-1) < phi(k) => Bpos[k](i,j) = 0
        // Bpos[k](i,j) = 1 => R(i,j) >= phi(k) > phi(k-1) => Bpos[k-1](i,j) = 1
        // Thus Bpos[k](i, j) <= Bpos[k-1](i, j)
        prog->AddLinearConstraint(Bpos[k](i, j) <= Bpos[k-1](i, j));

        // Bneg[k](i,j) = 1 => -R(i,j) >= phi(k) > phi(k-1) => Bneg[k-1](i,j) = 1
        // Bneg[k-1](i,j) = 0 => -R(i,j) <= phi(k-1) < phi(k) => Bneg[k](i,j) = 0
        // Thus Bneg[k](i,j) <= Bneg[k-1](i,j)
        prog->AddLinearConstraint(Bneg[k](i, j) <= Bneg[k-1](i, j));

        for (int l = 1; l < num_binary_vars_per_half_axis; ++l) {
          // Either R(i, j) >= phi(k) or R(i, j) <= -phi(l).
          // So Bpos[k](i,j) + Bneg[l](i,j) <= 1
          prog->AddLinearConstraint(Bpos[k](i, j) + Bneg[l](i, j) <= 1);
        }
      }
    }
  }

  // Add angle limit constraints.
  // Bounding box will turn on/off an orthant.  It's sufficient to add the
  // constraints only to the positive orthant.
  AddBoundingBoxConstraintsImpliedByRollPitchYawLimitsToBinary(prog, Bpos[0],
                                                               limits);

  // Add constraints to the column and row vectors.
  std::vector<MatrixDecisionVariable<3, 1>> cpos(num_binary_vars_per_half_axis),
      cneg(num_binary_vars_per_half_axis);
  for (int i = 0; i < 3; i++) {
    // Make lists of the decision variables in terms of column vectors and row
    // vectors to facilitate the calls below.
    // TODO(russt): Consider reorganizing the original Cpos/Cneg variables to
    // avoid this (albeit minor) cost?
    for (int k = 0; k < num_binary_vars_per_half_axis; k++) {
      cpos[k] = Cpos[k].col(i);
      cneg[k] = Cneg[k].col(i);
    }
    AddMcCormickVectorConstraints(prog, R.col(i), cpos, cneg,
                                  R.col((i + 1) % 3), R.col((i + 2) % 3));

    for (int k = 0; k < num_binary_vars_per_half_axis; k++) {
      cpos[k] = Cpos[k].row(i).transpose();
      cneg[k] = Cneg[k].row(i).transpose();
    }
    AddMcCormickVectorConstraints(prog, R.row(i).transpose(), cpos, cneg,
                                  R.row((i + 1) % 3).transpose(),
                                  R.row((i + 2) % 3).transpose());
  }
}

}  // namespace solvers
}  // namespace drake
