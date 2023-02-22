#include "drake/geometry/optimization/c_iris_collision_geometry.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"

namespace drake {
namespace geometry {
namespace optimization {

CIrisCollisionGeometry::CIrisCollisionGeometry(const Shape* geometry,
                                               multibody::BodyIndex body_index,
                                               GeometryId id,
                                               math::RigidTransformd X_BG)
    : geometry_{geometry}, body_index_{body_index}, id_{id}, X_BG_{X_BG} {}

namespace {
struct ReifyData {
  // To avoid copying objects (which might be expensive), I store the
  // non-primitive-type objects with pointers.
  const Vector3<symbolic::Polynomial>* a{};
  const symbolic::Polynomial* b{};
  const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>*
      X_AB_multilinear{};
  const multibody::RationalForwardKinematics* rational_forward_kin{};
  const PlaneSide plane_side;
  const GeometryId geometry_id;
  const VectorX<symbolic::Variable>* y_slack{};
  std::vector<symbolic::RationalFunction>* rationals{};
};

// Compute the rational function
//  (aᵀ*p_AQ + b) - offset if plane_side = kPositive
// -(aᵀ*p_AQ + b) - offset if plane_side = kNegative
// @param a_A The vector a measured in the frame A.
// @param b The constant term in the separating plane.
// @param p_GQ The position of the point Q in the geometry frame G.
// @param X_BG The pose of the geometry frame G in the body frame B.
// @param X_AB_multilinear The pose of body frame B expressed in frame A.
// @param plane_side The side of the plane where Q lives.
[[nodiscard]] symbolic::RationalFunction ComputePointOnPlaneSideRational(
    const Vector3<symbolic::Polynomial>& a_A, const symbolic::Polynomial& b,
    const Eigen::Ref<const Eigen::Vector3d>& p_GQ,
    const math::RigidTransformd& X_BG,
    const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
        X_AB_multilinear,
    double offset, PlaneSide plane_side,
    const multibody::RationalForwardKinematics& rational_forward_kin) {
  // First compute p_AQ.
  const Eigen::Vector3d p_BQ = X_BG * p_GQ;
  const Vector3<symbolic::Polynomial> p_AQ =
      X_AB_multilinear.position + X_AB_multilinear.rotation * p_BQ;
  // Compute lhs=aᵀ*p_AQ + b
  const symbolic::Polynomial lhs = a_A.dot(p_AQ) + b;
  if (plane_side == PlaneSide::kPositive) {
    return rational_forward_kin.ConvertMultilinearPolynomialToRationalFunction(
        lhs - offset);
  } else {
    return rational_forward_kin.ConvertMultilinearPolynomialToRationalFunction(
        -offset - lhs);
  }
}

void ImplementPointRationals(const Eigen::Ref<const Eigen::Matrix3Xd>& p_GV,
                             const math::RigidTransformd& X_BG,
                             ReifyData* reify_data) {
  // As part of the conditions that a geometry is on one side of the separating
  // plane, we impose the following constraint:
  // if plane_side = kPositive
  //   aᵀ*p_AV+b ≥ 1
  // if plane_side = kNegative
  //   aᵀ*p_AV+b ≤ -1

  const double offset{1};
  const int num_rationals_for_points = p_GV.cols();
  reify_data->rationals->reserve(reify_data->rationals->size() +
                                 num_rationals_for_points);
  // The position of the vertices V in the geometry frame G.
  for (int i = 0; i < p_GV.cols(); ++i) {
    reify_data->rationals->push_back(ComputePointOnPlaneSideRational(
        *(reify_data->a), *(reify_data->b), p_GV.col(i), X_BG,
        *(reify_data->X_AB_multilinear), offset, reify_data->plane_side,
        *(reify_data->rational_forward_kin)));
  }
}

void ImplementSpherePsdMatRational(const Eigen::Vector3d& p_GS,
                                   const math::RigidTransformd& X_BG,
                                   double radius, ReifyData* reify_data) {
  // As part of the condition that a sphere-based geometry (e.g., sphere,
  // capsule) is on one side of the separating plane, we impose the constraint
  // aᵀ*p_AS + b ≥ r|a|       if plane_side = kPositive   (1)
  // aᵀ*p_AS + b ≤ -r|a|      if plane_side = kNegative   (2)
  // (1) means that if plane_side = kPositive, the matrix
  // ⌈aᵀp_AS + b                aᵀ⌉  is psd.           (3)
  // ⌊ a        (aᵀp_AS + b)/r²*I₃⌋
  // (3) is equivalent to the rational
  // ⌈1⌉ᵀ⌈aᵀp_AS + b               aᵀ⌉⌈1⌉
  // ⌊y⌋ ⌊ a        (aᵀp_AS+ b)/r²*I₃⌋⌊y⌋
  // being non-negative.
  // Likewise if plane_side = kNegative, the matrix
  // ⌈-aᵀp_AS - b                aᵀ⌉  is psd.           (4)
  // ⌊ a        -(aᵀp_AS + b)/r²*I₃⌋
  // (4) is equivalent to the rational
  // ⌈1⌉ᵀ⌈-aᵀp_AS - b               aᵀ⌉⌈1⌉
  // ⌊y⌋ ⌊ a        -(aᵀp_AS+ b)/r²*I₃⌋⌊y⌋
  // being non-negative.
  const Eigen::Vector3d p_BS = X_BG * p_GS;
  const Vector3<symbolic::Polynomial> p_AS =
      reify_data->X_AB_multilinear->position +
      reify_data->X_AB_multilinear->rotation * p_BS;
  // Compute aᵀp_AS + b
  const symbolic::RationalFunction a_dot_x_plus_b =
      reify_data->rational_forward_kin
          ->ConvertMultilinearPolynomialToRationalFunction(
              reify_data->a->dot(p_AS) + *(reify_data->b));
  const Vector3<symbolic::Polynomial> y_poly(
      symbolic::Polynomial((*(reify_data->y_slack))(0)),
      symbolic::Polynomial((*(reify_data->y_slack))(1)),
      symbolic::Polynomial((*(reify_data->y_slack))(2)));
  // Compute yᵀy
  const symbolic::Polynomial y_squared{
      {{symbolic::Monomial((*(reify_data->y_slack))(0), 2), 1},
       {symbolic::Monomial((*(reify_data->y_slack))(1), 2), 1},
       {symbolic::Monomial((*(reify_data->y_slack))(2), 2), 1}}};
  const int sign = reify_data->plane_side == PlaneSide::kPositive ? 1 : -1;
  reify_data->rationals->emplace_back(
      sign * a_dot_x_plus_b.numerator() +
          2 * reify_data->a->dot(y_poly) * a_dot_x_plus_b.denominator() +
          y_squared / (radius * radius) * sign * a_dot_x_plus_b.numerator(),
      a_dot_x_plus_b.denominator());
}

class OnPlaneSideReifier : public ShapeReifier {
 public:
  OnPlaneSideReifier(const Shape* geometry, math::RigidTransformd X_BG,
                     GeometryId geometry_id)
      : geometry_{geometry},
        X_BG_{std::move(X_BG)},
        geometry_id_{geometry_id} {}

  void ProcessData(
      const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
      const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
          X_AB_multilinear,
      const multibody::RationalForwardKinematics& rational_forward_kin,
      PlaneSide plane_side, const VectorX<symbolic::Variable>& y_slack,
      std::vector<symbolic::RationalFunction>* rationals) {
    ReifyData data{
        &a,         &b,           &X_AB_multilinear, &rational_forward_kin,
        plane_side, geometry_id_, &y_slack,          rationals};
    geometry_->Reify(this, &data);
  }

 private:
  // Implements supported shapes for the ShapeReifier interface.
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box& box, void* data) {
    auto* reify_data = static_cast<ReifyData*>(data);
    // The position of the vertices V in the geometry frame G.
    Eigen::Matrix<double, 3, 8> p_GV;
    // clang-format off
    p_GV << 1, 1, 1, 1, -1, -1, -1, -1,
            1, 1, -1, -1, 1, 1, -1, -1,
            1, -1, 1, -1, 1, -1, 1, -1;
    // clang-format on
    p_GV.row(0) *= box.width() / 2;
    p_GV.row(1) *= box.depth() / 2;
    p_GV.row(2) *= box.height() / 2;
    ImplementPointRationals(p_GV, X_BG_, reify_data);
  }

  void ImplementGeometry(const Convex& convex, void* data) {
    auto* reify_data = static_cast<ReifyData*>(data);
    const Eigen::Matrix3Xd p_GV = GetVertices(convex);
    ImplementPointRationals(p_GV, X_BG_, reify_data);
  }

  void ImplementGeometry(const Sphere& sphere, void* data) {
    auto* reify_data = static_cast<ReifyData*>(data);
    // If the sphere with radius r is on one side of the plane
    // it is equivalent to the following condition
    // aᵀ*p_AS + b ≥ r|a|       if plane_side = kPositive   (1a)
    // aᵀ*p_AS + b ≥ 1          if plane_side = kPositive   (1b)
    //
    // aᵀ*p_AS + b ≤ -r|a|      if plane_side = kNegative   (2a)
    // aᵀ*p_AS + b ≤ -1         if plane_side = kNegative   (2b)
    // where S is the center of the sphere. p_AS is the position of S expressed
    // in the frame A where the separating plane is also expressed.

    // First add the psd matrix constraint.
    ImplementSpherePsdMatRational(Eigen::Vector3d::Zero(), X_BG_,
                                  sphere.radius(), reify_data);
    // Now add the rational constraint
    // aᵀ*p_AS + b ≥ 1          if plane_side = kPositive   (1b)
    // aᵀ*p_AS + b ≤ -1         if plane_side = kNegative   (2b)
    ImplementPointRationals(Eigen::Vector3d::Zero(), X_BG_, reify_data);
  }

  void ImplementGeometry(const Capsule& capsule, void* data) {
    // If the capsule with radius r is on one side of the plane
    // it is equivalent to the following condition
    // If plane_side = kPositive
    // aᵀ*p_AS1 + b ≥ r|a|        (1a)
    // aᵀ*p_AS2 + b ≥ r|a|        (1b)
    // aᵀ*p_AO + b ≥ 1            (1c)
    //
    // If plane_side = kNegative
    // aᵀ*p_AS1 + b ≤ -r|a|       (2a)
    // aᵀ*p_AS2 + b ≤ -r|a|       (2b)
    // aᵀ*p_AO + b ≤ -1           (2c)
    // where S1 and S2 are the center of the two spheres, O is the center of
    // the capsule.
    // Please refer to our implementation for sphere to understand how we
    // impose conditions (1) and (2).
    //
    // Add the psd-mat constraints.
    auto* reify_data = static_cast<ReifyData*>(data);
    ImplementSpherePsdMatRational(Eigen::Vector3d(0, 0, capsule.length() / 2),
                                  X_BG_, capsule.radius(), reify_data);
    ImplementSpherePsdMatRational(Eigen::Vector3d(0, 0, -capsule.length() / 2),
                                  X_BG_, capsule.radius(), reify_data);
    // aᵀ*p_AO + b ≥ 1          if plane_side = kPositive
    // aᵀ*p_AO + b ≤ -1         if plane_side = kNegative
    ImplementPointRationals(Eigen::Vector3d::Zero(), X_BG_, reify_data);
  }

  void ImplementGeometry(const Cylinder& cylinder, void* data) {
    // Consider a cylinder with radius r and length l, if this cylinder is on
    // the positive side of a plane {x | aᵀx + b=0}, we first write out the
    // condition on the plane parameter in the cylinder geometry frame G, and
    // then derive the condition on the plane parameter in the "expressed frame"
    // A, where A can be any frame.
    //
    // In the cylinder's geometry frame G, the cylinder center is at the frame
    // origin, and the cylinder axis is along the frame z axis. This cylinder is
    // on the positive side of the plane { p_GQ | a_Gᵀ*p_GQ + b_G = 0} if and
    // only if both the top and bottom rims of the cylinder are on the positive
    // side of the plane, namely a_G.dot([r*cosθ, r*sinθ, ±l/2]) + b_G ≥ 0 for
    // any θ. Minimizing over θ implies that this condition is equivalent to
    //  a_G(2) * l/2 + b_G ≥ r * |[a_G(0), a_G(1)]|    (1a)
    // -a_G(2) * l/2 + b_G ≥ r * |[a_G(0), a_G(1)]|    (1b)
    // We also impose the constraint a_G.dot(p_GO) + b_G >= 1 to exclude the
    // trivial solution a_G=0, b_G=0, where O is the center of the cylinder,
    // namely G's frame origin. This is equivalent to
    // b_G ≥ 1       (2)
    //
    // Similarly if the cylinder is on the negative side of the plane, we
    // impose the constraint
    //  a_G(2) * l/2 + b_G ≤ -r * |[a_G(0), a_G(1)]|    (3a)
    // -a_G(2) * l/2 + b_G ≤ -r * |[a_G(0), a_G(1)]|    (3b)
    // and
    // b_G ≤ -1      (4)
    //
    // Notice that (1a) is equivalent to the following matrix being psd.
    // ⌈a_G(2)*l/2+b_G     a_G(0)                a_G(1) ⌉
    // |a_G(0)      (a_G(2)*l/2+b_G)/r²               0 |  ≽ 0  (5a)
    // ⌊a_G(1)              0        (a_G(2)*l/2+b_G)/r²⌋
    // Likewise we can formulate (1b) as another matrix being psd.
    //
    // The matrix in (5a) is psd if and only if the following function is always
    // non-negative
    // ⌈  1 ⌉ᵀ⌈a_G(2)*l/2+b_G   a_G(0)              a_G(1) ⌉⌈  1 ⌉
    // |y(0)| |a_G(0)    (a_G(2)*l/2+b_G)/r²             0 ||y(0)| ≥ 0 (6a)
    // ⌊y(1)⌋ ⌊a_G(1)            0      (a_G(2)*l/2+b_G)/r²⌋⌊y(1)⌋
    //
    // We derived the condition (6a) and (2) using the plane expressed in
    // cylinder geometry frame (with parameter a_G, b_G). We next derive the
    // condition using the plane expressed in an arbitrary frame A (with
    // parameter a_A, b_A), and the SE(3) transformation between the two frames
    // X_AG, and show that (6a) and (2) are both rational functions of s.
    //
    // Here we derive the relationship between (a_A, b_A) and (a_G, b_G).
    // For a point Q on the plane, it satisfies the condition
    // a_Gᵀ * p_GQ + b_G = 0            (7)
    // and also we can write the condition in the A's frame.
    // a_Aᵀ * p_AQ + b_A = 0            (8)
    // Since we know that
    // p_AQ = R_AG * p_GQ + p_AG          (9)
    // Substitute (9) into (8) we have
    // (a_Aᵀ * R_AG) * p_GQ + (b_A + a_Aᵀ * p_AG) = 0  (10)
    // Comparing (7) and (10), we have
    // a_G = R_AGᵀ * a_A                   (11a)
    // b_G = b_A + a_Aᵀ * p_AG             (11b)
    // Hence we can write the pair (a_G, b_G) as a function of (a_A, b_A) and
    // the SE(3) transform X_AG. Now substitute (11a) (11b) to (6a) and (2)
    // we get the rationals that need to be non-negative.

    auto* reify_data = static_cast<ReifyData*>(data);

    // a_G = R_GA * a_A
    //     = (R_AB * R_BG)ᵀ * a_A
    const Vector3<symbolic::Polynomial> a_G =
        (reify_data->X_AB_multilinear->rotation * X_BG_.rotation().matrix())
            .transpose() *
        (*reify_data->a);
    // p_AG = R_AB * p_BG + p_AB
    const Vector3<symbolic::Polynomial> p_AG =
        reify_data->X_AB_multilinear->rotation * X_BG_.translation() +
        reify_data->X_AB_multilinear->position;
    const symbolic::Polynomial b_G = *reify_data->b + reify_data->a->dot(p_AG);
    switch (reify_data->plane_side) {
      case PlaneSide::kPositive: {
        // impose the constraint b_G >= 1
        reify_data->rationals->push_back(
            reify_data->rational_forward_kin
                ->ConvertMultilinearPolynomialToRationalFunction(b_G - 1));
        break;
      }
      case PlaneSide::kNegative: {
        // Impose the constraint b_G <= -1
        reify_data->rationals->push_back(
            reify_data->rational_forward_kin
                ->ConvertMultilinearPolynomialToRationalFunction(-1 - b_G));
        break;
      }
    }

    // Now compute the polynomial
    // ⌈  1 ⌉ᵀ⌈a_G(2)*l/2+b_G   a_G(0)              a_G(1) ⌉⌈  1 ⌉
    // |y(0)| |a_G(0)    (a_G(2)*l/2+b_G)/r²             0 ||y(0)|
    // ⌊y(1)⌋ ⌊a_G(1)            0      (a_G(2)*l/2+b_G)/r²⌋⌊y(1)⌋
    const Vector2<symbolic::Polynomial> y_poly(
        symbolic::Polynomial(symbolic::Monomial((*reify_data->y_slack)(0))),
        symbolic::Polynomial(symbolic::Monomial((*reify_data->y_slack)(1))));
    // Compute y(0)*y(0) + y(1)*y(1)
    const symbolic::Polynomial y_squared{
        {{symbolic::Monomial((*reify_data->y_slack)(0), 2), 1},
         {symbolic::Monomial((*reify_data->y_slack)(1), 2), 1}}};
    const int plane_sign =
        reify_data->plane_side == PlaneSide::kPositive ? 1 : -1;
    for (double scalar : {1, -1}) {
      // scalar = 1 for the top rim, and scalar = -1 for the bottom rim.
      // Compute plane_sign * (±l/2 * a_G(2) + b_G)
      const symbolic::Polynomial a2_plus_b =
          plane_sign * (scalar * cylinder.length() / 2 * a_G(2) + b_G);
      reify_data->rationals->push_back(
          reify_data->rational_forward_kin
              ->ConvertMultilinearPolynomialToRationalFunction(
                  a2_plus_b + 2 * y_poly(0) * a_G(0) + 2 * y_poly(1) * a_G(1) +
                  a2_plus_b / (cylinder.radius() * cylinder.radius()) *
                      y_squared));
    }
  }

  const Shape* geometry_;
  math::RigidTransformd X_BG_;
  GeometryId geometry_id_;
};

class CIrisGeometryTypeReifier : public ShapeReifier {
 public:
  explicit CIrisGeometryTypeReifier(const Shape& shape) { shape.Reify(this); }

  CIrisGeometryType type() const { return type_; }

 private:
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Box&, void*) {
    type_ = CIrisGeometryType::kPolytope;
  }

  void ImplementGeometry(const Convex&, void*) {
    type_ = CIrisGeometryType::kPolytope;
  }

  void ImplementGeometry(const Sphere&, void*) {
    type_ = CIrisGeometryType::kSphere;
  }

  void ImplementGeometry(const Capsule&, void*) {
    type_ = CIrisGeometryType::kCapsule;
  }

  void ImplementGeometry(const Cylinder&, void*) {
    type_ = CIrisGeometryType::kCylinder;
  }

  CIrisGeometryType type_;
};

class NumRationalsReifier : public ShapeReifier {
 public:
  explicit NumRationalsReifier(const Shape& shape) { shape.Reify(this); }

  int count() const { return count_; }

 private:
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box&, void*) {
    // We implement a.dot(p_AVᵢ) + b >= 1 (or <= -1) where Vᵢ is the vertex of
    // the box. The box has 8 vertices.
    count_ = 8;
  }

  void ImplementGeometry(const Convex& convex, void*) {
    // One rational for each vertex of the polytope.
    const Eigen::Matrix3Xd p_GV = GetVertices(convex);
    count_ = p_GV.cols();
  }

  void ImplementGeometry(const Sphere&, void*) {
    // Two rationals:
    // a.dot(p_AS) + b >= r * |a|  (or <= -r * |a|)
    // a.dot(p_AS) + b >= 1        (or <= -1)
    count_ = 2;
  }

  void ImplementGeometry(const Capsule&, void*) {
    // Three rationals
    // a.dot(p_AS1) + b >= r * |a|  (or <= -r * |a|)
    // a.dot(p_AS2) + b >= r * |a|  (or <= -r * |a|)
    // a.dot(p_AO) + b >= 1        (or <= -1)
    count_ = 3;
  }

  void ImplementGeometry(const Cylinder&, void*) {
    // Three rationals
    //  a_G(2)*h/2+b_G >= r*|[a_G(0) a_G(1)]| (or <= -r*|[a_G(0) a_G(1)]|)
    // -a_G(2)*h/2+b_G >= r*|[a_G(0) a_G(1)]| (or <= -r*|[a_G(0) a_G(1)]|)
    //  b_G >= 1 (or <= -1).
    count_ = 3;
  }

  int count_;
};

// Compute the signed distance from a collision geometry to the half space
// { p_GQ | a_Gᵀ*p_GQ+b_G >= 0}, where a_G is in the collision geometry frame.
class DistanceToHalfspaceReifier : public ShapeReifier {
 public:
  explicit DistanceToHalfspaceReifier(const Shape& shape, Eigen::Vector3d a_G,
                                      double b_G)
      : a_G_{std::move(a_G)}, b_G_{b_G} {
    shape.Reify(this);
  }

  double distance() const { return distance_; }

 private:
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box& box, void*) {
    // Box vertices.
    Eigen::Matrix<double, 3, 8> p_GV;
    // clang-format off
    p_GV << -1, -1, -1, -1, 1, 1, 1, 1,
            1, 1, -1, -1, 1, 1, -1, -1,
            1, -1, 1, -1, 1, -1, 1, -1;
    // clang-format on
    p_GV.row(0) *= box.width() / 2;
    p_GV.row(1) *= box.depth() / 2;
    p_GV.row(2) *= box.height() / 2;
    distance_ = ((a_G_.transpose() * p_GV).minCoeff() + b_G_) / (a_G_.norm());
  }

  void ImplementGeometry(const Convex& convex, void*) {
    distance_ = ((a_G_.transpose() * GetVertices(convex)).minCoeff() + b_G_) /
                (a_G_.norm());
  }

  void ImplementGeometry(const Sphere& sphere, void*) {
    distance_ = b_G_ / (a_G_.norm()) - sphere.radius();
  }

  void ImplementGeometry(const Capsule& capsule, void*) {
    Eigen::Matrix<double, 3, 2> p_GS;
    p_GS.col(0) << 0, 0, capsule.length() / 2;
    p_GS.col(1) << 0, 0, -capsule.length() / 2;
    distance_ = ((a_G_.transpose() * p_GS).minCoeff() + b_G_) / (a_G_.norm()) -
                capsule.radius();
  }

  void ImplementGeometry(const Cylinder& cylinder, void*) {
    // The distance from a point on the top rim [r*cosθ, r*sinθ, l/2] to the
    // face {x | a.dot(x)+b=0} is (a(0)*r*cosθ + a(1)*r*sinθ + a(2)*l/2 + b) /
    // |a|. Taking the minimum over θ, we get the distance from the top rim to
    // the face as (-r * |[a(0), a(1)]| + a(2)*l/2 + b) / |a|. Similarly, the
    // distance from the bottom rim [r*cosθ, r*sinθ, -l/2] to the face {x |
    // a.dot(x)+b=0} is
    // (-r * |[a(0), a(1)]| - a(2)*l/2 + b) / |a|.
    distance_ =
        (-cylinder.radius() * a_G_.head<2>().norm() +
         (a_G_(2) >= 0 ? -a_G_(2) : a_G_(2)) * cylinder.length() / 2 + b_G_) /
        a_G_.norm();
  }

  Eigen::Vector3d a_G_;
  double b_G_;
  double distance_;
};
}  // namespace

void CIrisCollisionGeometry::OnPlaneSide(
    const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
    const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
        X_AB_multilinear,
    const multibody::RationalForwardKinematics& rational_forward_kin,
    PlaneSide plane_side, const VectorX<symbolic::Variable>& y_slack,
    std::vector<symbolic::RationalFunction>* rationals) const {
  OnPlaneSideReifier reifier(geometry_, X_BG_, id_);
  reifier.ProcessData(a, b, X_AB_multilinear, rational_forward_kin, plane_side,
                      y_slack, rationals);
}

CIrisGeometryType CIrisCollisionGeometry::type() const {
  CIrisGeometryTypeReifier reifier(*geometry_);
  return reifier.type();
}

int CIrisCollisionGeometry::num_rationals() const {
  NumRationalsReifier reifier(*geometry_);
  return reifier.count();
}

double DistanceToHalfspace(const CIrisCollisionGeometry& collision_geometry,
                           const Eigen::Vector3d& a, double b,
                           multibody::BodyIndex expressed_body,
                           PlaneSide plane_side,
                           const multibody::MultibodyPlant<double>& plant,
                           const systems::Context<double>& plant_context) {
  DRAKE_ASSERT((a.array() != 0).any());
  // Transforms the halfspace from frame E to the collision_geometry
  // geometry frame G. First compute the pose of frame E to the
  // collision geometry body frame B.
  const auto X_BE = plant.CalcRelativeTransform(
      plant_context,
      plant.get_body(collision_geometry.body_index()).body_frame(),
      plant.get_body(expressed_body).body_frame());
  const auto X_GE = collision_geometry.X_BG().inverse() * X_BE;
  // Compute the outward normal a_G and constant b_G of the halfspace in the G
  // frame. If we have a point Q on the plane, namely a_E.dot(p_EQ) + b_E = 0,
  // we want to express this point Q in the G frame as p_GQ = R_GE * p_EQ +
  // p_GE, and compute a_G, b_G such that a_G.dot(p_GQ) + b_G = 0.
  // We have a_G = R_GE * a_E, a_G.dot(p_GE) + b_G = b_E
  Eigen::Vector3d a_G = X_GE.rotation() * a;
  double b_G = b - a_G.dot(X_GE.translation());
  if (plane_side == PlaneSide::kNegative) {
    a_G = -a_G;
    b_G = -b_G;
  }

  DistanceToHalfspaceReifier reifier(collision_geometry.geometry(), a_G, b_G);
  return reifier.distance();
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
