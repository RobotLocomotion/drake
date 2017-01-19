#include "drake/solvers/rotation_constraint.h"
#include "drake/solvers/rotation_constraint_internal.h"

#include <algorithm>
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
  // Implemented as a rotated Lorenz cone using z = Ax+b = [ 1; 2; v1+v2 ].
  A.topRows<2>() = Eigen::Matrix<double, 2, 6>::Zero();
  A.bottomRows<3>() << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity();
  b << 1, 2, 0, 0, 0;
  prog->AddRotatedLorentzConeConstraint(A, b, {v1, v2});

  // |v1-v2|^2 <= 2
  // Implemented as a rotated Lorenz cone using z = Ax+b = [ 1; 2; v1-v2 ].
  A.block<3, 3>(2, 3) = -Eigen::Matrix3d::Identity();
  prog->AddRotatedLorentzConeConstraint(A, b, {v1, v2});
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

std::vector<Eigen::Vector3d> IntersectBoxWUnitCircle(Eigen::Vector3d bmin,
                                                     Eigen::Vector3d bmax) {
  // Assumes the positive orthant (and bmax>=bmin).
  DRAKE_ASSERT(bmin(0) >= 0 && bmin(1) >= 0 && bmin(2) >= 0);
  DRAKE_ASSERT(bmax(0) >= bmin(0) && bmax(1) >= bmin(1) && bmax(2) >= bmin(2));

  // Assumes the unit circle intersects the box.
  DRAKE_ASSERT(bmin.lpNorm<2>() <= 1);
  DRAKE_ASSERT(bmax.lpNorm<2>() >= 1);

  std::vector<Eigen::Vector3d> intersections;

  // Note: all logic below are ordered to avoid imaginary sqrts, but
  // the Intercept method asserts for this, just in case.

  // An axis-aligned box in the positive orthant can intersect with the unit
  // circle at:
  //  1 point - when the box touchest the unit circle exactly at the corner,
  //  3 points - when exactly one corner is inside the unit circle OR exactly
  //      one corner is outside the unit circle, or
  //  4 points, when >= two points are inside the unit circle and >= two
  //      points are outside the unit circle.

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

  // Finds the "bottom" (min z) intersection(s).
  Eigen::Vector3d v;
  v << bmax(0), bmax(1), bmin(2);  // far bottom corner
  if (v.lpNorm<2>() > 1) {
    // Then two intersections on the bottom face.
    // Get the +x one first.
    v = bmin;
    v(0) = Intercept(v(1), v(2));
    if (v(0) <= bmax(0)) {
      intersections.push_back(v);
    } else {
      // Must be on the back face.
      v(0) = bmax(0);
      v(1) = Intercept(v(0), v(2));
      intersections.push_back(v);
    }
    // Now get the +y intersection.
    v = bmin;
    v(1) = Intercept(v(0), v(2));
    if (v(1) <= bmax(1)) {
      intersections.push_back(v);
    } else {
      v(1) = bmax(1);
      v(0) = Intercept(v(1), v(2));
      intersections.push_back(v);
    }
  } else {
    // Then exactly one intersection on the bmax(0),bmax(1),z edge.
    DRAKE_ASSERT(v(0) == bmax(0) && v(1) == bmax(1));  // already set for me.
    v(2) = Intercept(v(0), v(1));
    intersections.push_back(v);
  }

  // Finds the "top" (max z) intersections(s).
  v << bmin(0), bmin(1), bmax(2);  // near top corner
  if (v.lpNorm<2>() >= 1) {
    // Then exact one "top" intersection, along this edge.
    v(2) = Intercept(v(0), v(1));
    intersections.push_back(v);
  } else {
    // Then exactly two intersections on the top face.
    v(0) = Intercept(v(1), v(2));
    if (v(0) <= bmax(0)) {
      intersections.push_back(v);
    } else {
      v(0) = bmax(0);
      v(1) = Intercept(v(0), v(2));
      intersections.push_back(v);
    }

    v << bmin(0), bmin(1), bmax(2);
    v(1) = Intercept(v(0), v(2));
    if (v(1) <= bmax(1)) {
      intersections.push_back(v);
    } else {
      v(1) = bmax(1);
      v(0) = Intercept(v(1), v(2));
      intersections.push_back(v);
    }
  }

  if (intersections.size() == 2) {
    // Then the unit circle passed through the near and far vertical edges,
    // and missed the top and bottom faces.  There must be two more
    // intersections -- on the other two vertical edges.

    v << bmin(0), bmax(1), Intercept(bmin(0), bmax(1));
    intersections.push_back(v);

    v << bmax(0), bmin(1), Intercept(bmax(0), bmin(1));
    intersections.push_back(v);
  }
  return intersections;
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
            //    d <= normal'*v
            // that puts v inside (but as close as possible to) the unit circle.
            auto pts = internal::IntersectBoxWUnitCircle(box_min, box_max);
            DRAKE_DEMAND(pts.size() >= 3);
            Eigen::Vector3d normal;
            // Note: 1-d is the distance to the farthest point on the unit
            // circle that is inside the bounding box, so intentionally
            // initialize it to a (known) bad value.
            double d = -1;

            if (pts.size() == 3) {
              normal = (pts[1] - pts[0]).cross(pts[2] - pts[0]);
              if (normal(0) < 0) normal = -normal;
              normal.normalize();
              d = normal.dot(pts[0]);
            } else {
              // 4 points, so search for the tightest linear constraint.
              // (will intersect with 3 of the points, so just try all
              // combinations).
              Eigen::Vector3d n;
              for (int i = 0; i < 4; i++) {
                n = (pts[(1 + i) % 4] - pts[i])
                        .cross(pts[(2 + i) % 4] - pts[i]);
                if (n(0) < 0) n = -n;
                n.normalize();
                const double this_d = n.dot(pts[i]);
                if (n.dot(pts[(3 + i) % 4]) >=
                        this_d  // then it's a valid constraint
                    &&
                    this_d > d) {  // and it's tighter than the previous best.
                  normal = n;
                  d = this_d;
                }
              }
              DRAKE_DEMAND(d >= 0 && d <= 1);
            }
            DRAKE_DEMAND(normal(0) > 0 && normal(1) > 0 && normal(2) > 0);

            // Useful below: the intersection points of the unit sphere with
            // the box represent the convex hull of directions for vector v.
            // Since all points must have magnitude 1 (and we've normalized the
            // normal), all vectors within this set are within an angle theta
            // of the normal, with
            //    cos(theta) = min_i normal.dot(pt[i]),
            // and we have 0 <= theta < pi/2.
            // Proof sketch:
            // Every vector in the convex hull of the intersection points
            // can be written as
            //   v = sum_i w_i p_i, w_i>0, sum_i w_i=1.
            // Note that |v| = 1 when v=p_i, and <1 when v is inside the hull.
            // Furthermore, every point in the intersection of the bounding box
            // and the unit circle can be represented by a vector u which is
            // a vector v in the convex hull, but with length normalized to 1.
            //   u = v / |v|.
            // Given normal'*u = |normal||u|cos(theta), and |normal|=|u|=1,
            // we have
            //   cos(theta) = normal'*v/|v| = (\sum w_i normal'*p_i)/|v|.
            // This obtains a minimum when w_i=1 for min_i normal'*p_i,
            // (because that maximizes the denominator AND minimizes the
            //  numerator), yielding:
            //   cos(theta) >= min_i normal.dot(pt[i]).
            double cos_theta = 1;
            for (const auto& pt : pts) {
              cos_theta = std::min(cos_theta, normal.dot(pt));
            }
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
              // Since v is within theta of normal, we have that v2 must be
              // within theta of normal.cross(v1).  To see this, observe that
              //   v2'*(normal.cross(v1)) = normal'*(v1.cross(v2))
              //     = normal'*v >= cos(theta).
              // The vector normal.cross(v1) has a length less than 1, so
              //   v2'*(normal.cross(v1)/|normal.cross(v1)|) >=
              //     v2'*(normal.cross(v1)) >= cos(theta)
              // Thus the angle between the vector v2 and normal.cross(v1) is
              // less than theta.  In fact this is very conservative -- v2 can
              // actually only rotate by theta in the plane orthogonal to v1.
              // Since the maximum slope of sin() is maximal around 0, this
              // confers a conservative elementwise convex(!) constraint that
              //   -asin(theta) <= v2 - normal.cross(v1) <= asin(theta).
              // Since 0<=theta<=pi/2, this should be enough to rule out the
              // det(R)=-1 case (the shortest projection of a line across the
              // circle onto a single axis has length 2sqrt(3)/3 > 1.15), and
              // can be significantly tighter.

              // To activate this only when the box is active, the complete
              // constraints are
              //  -asin(theta)-6+2(cxi+cyi+czi) <= v2-normal.cross(v1)
              //    v2-normal.cross(v1) <= asin(theta)+6-2(cxi+cyi+czi)
              // Note: Again this constraint could be tighter as a Lorenz cone
              // constraint of the form:
              //   |v2 - normal.cross(v1)| <= 2*sin(theta/2).
              A_cross << Eigen::Matrix3d::Identity(),
                  -math::VectorToSkewSymmetric(orthant_normal),
                  Eigen::Matrix3d::Constant(-2);
              prog->AddLinearConstraint(
                  A_cross, Eigen::Vector3d::Constant(-std::asin(theta) - 6),
                  Eigen::Vector3d::Constant(2), {v2, v1, orthant_c});

              A_cross.rightCols<3>() = Eigen::Matrix3d::Constant(2.0);
              prog->AddLinearConstraint(
                  A_cross, Eigen::Vector3d::Constant(-2),
                  Eigen::Vector3d::Constant(std::asin(theta) + 6),
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
        prog->AddLinearConstraint(
            Eigen::RowVector2d(1, 1), -phi(k), 2 - phi(k),
            VectorDecisionVariable<2>{R(i, j), Bneg[k](i, j)});
        // Tight on the lower bound:
        prog->AddLinearConstraint(
            Eigen::RowVector2d(1, 2), -phi(k), 2 - phi(k),
            VectorDecisionVariable<2>{R(i, j), Bneg[k](i, j)});

        if (k == num_binary_vars_per_half_axis - 1) {
          //   Cpos[k](i,j) = Bpos[k](i,j)
          prog->AddLinearEqualityConstraint(
              Eigen::RowVector2d(1, -1), 0,
              {Cpos[k].block<1, 1>(i, j), Bpos[k].block<1, 1>(i, j)});
          //   Cneg[k](i,j) = Bneg[k](i,j)
          prog->AddLinearEqualityConstraint(
              Eigen::RowVector2d(1, -1), 0,
              {Cneg[k].block<1, 1>(i, j), Bneg[k].block<1, 1>(i, j)});
        } else {
          //   Cpos[k](i,j) = Bpos[k](i,j) - Bpos[k+1](i,j)
          prog->AddLinearEqualityConstraint(
              Eigen::RowVector3d(1, -1, 1), 0,
              {Cpos[k].block<1, 1>(i, j), Bpos[k].block<1, 1>(i, j),
               Bpos[k + 1].block<1, 1>(i, j)});
          //   Cneg[k](i,j) = Bneg[k](i,j) - Bneg[k+1](i,j)
          prog->AddLinearEqualityConstraint(
              Eigen::RowVector3d(1, -1, 1), 0,
              {Cneg[k].block<1, 1>(i, j), Bneg[k].block<1, 1>(i, j),
               Bneg[k + 1].block<1, 1>(i, j)});
        }
      }
      // Bpos[0](i,j) + Bneg[0](i,j) = 1.  (have to pick a side).
      prog->AddLinearEqualityConstraint(
          Eigen::RowVector2d(1, 1), 1,
          {Bpos[0].block<1, 1>(i, j), Bneg[0].block<1, 1>(i, j)});

      // for debugging: constrain to positive orthant.
      //      prog->AddBoundingBoxConstraint(1,1,{Bpos[0].block<1,1>(i,j)});
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
