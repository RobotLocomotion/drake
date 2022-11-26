#include "drake/geometry/optimization/dev/collision_geometry.h"

#include "drake/geometry/optimization/vpolytope.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"

namespace drake {
namespace geometry {
namespace optimization {

CollisionGeometry::CollisionGeometry(const geometry::Shape* geometry,
                                     multibody::BodyIndex body_index,
                                     geometry::GeometryId id,
                                     math::RigidTransformd X_BG)
    : geometry_{geometry}, body_index_{body_index}, id_{id}, X_BG_{X_BG} {}

namespace {
struct ReifyData {
  ReifyData(
      const Vector3<symbolic::Polynomial>& m_a, const symbolic::Polynomial& m_b,
      const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
          m_X_AB_multilinear,
      const multibody::RationalForwardKinematics& m_rational_forward_kin,
      const std::optional<symbolic::Variable>& m_separating_margin,
      PlaneSide m_plane_side,
      std::vector<symbolic::RationalFunction>* m_rationals,
      std::optional<VectorX<symbolic::Polynomial>>* m_unit_length_vector)
      : a{&m_a},
        b{&m_b},
        X_AB_multilinear{&m_X_AB_multilinear},
        rational_forward_kin{&m_rational_forward_kin},
        separating_margin{&m_separating_margin},
        plane_side{m_plane_side},
        rationals{m_rationals},
        unit_length_vector{m_unit_length_vector} {}

  // To avoid copying objects (which might be expensive), I store the
  // non-primitive-type objects with pointers.
  const Vector3<symbolic::Polynomial>* a;
  const symbolic::Polynomial* b;
  const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>*
      X_AB_multilinear;
  const multibody::RationalForwardKinematics* rational_forward_kin;
  const std::optional<symbolic::Variable>* separating_margin;
  const PlaneSide plane_side;
  std::vector<symbolic::RationalFunction>* rationals;
  std::optional<VectorX<symbolic::Polynomial>>* unit_length_vector;
};
}  // namespace

void CollisionGeometry::OnPlaneSide(
    const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
    const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
        X_AB_multilinear,
    const multibody::RationalForwardKinematics& rational_forward_kin,
    const std::optional<symbolic::Variable>& separating_margin,
    PlaneSide plane_side, std::vector<symbolic::RationalFunction>* rationals,
    std::optional<VectorX<symbolic::Polynomial>>* unit_length_vector) {
  ReifyData data(a, b, X_AB_multilinear, rational_forward_kin,
                 separating_margin, plane_side, rationals, unit_length_vector);
  geometry_->Reify(this, &data);
}

namespace {
// Compute the rational function
//  (aᵀ*p_AQ + b) - (offset + separating_margin) if plane_side = kPositive
// -(aᵀ*p_AQ + b) - (offset + separating_margin) if plane_side = kNegative
// @param a_A The vector a measured in the frame A.
// @param b The constant term in the separating plane.
// @param p_GQ The position of the point Q in the geometry frame G.
// @param X_BG The pose of the geometry frame G in the body frame B.
// @param X_AB_multilinear The pose of body frame B expressed in frame A.
// @param separating_margin If set to std::nullopt, then we ignore the
// separating margin.
// @param plane_side The side of the plane where Q lives.
[[nodiscard]] symbolic::RationalFunction ComputePointOnPlaneSideRational(
    const Vector3<symbolic::Polynomial>& a_A, const symbolic::Polynomial& b,
    const Eigen::Ref<const Eigen::Vector3d>& p_GQ,
    const math::RigidTransformd& X_BG,
    const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
        X_AB_multilinear,
    double offset, const std::optional<symbolic::Variable>& separating_margin,
    PlaneSide plane_side,
    const multibody::RationalForwardKinematics& rational_forward_kin) {
  // First compute p_AQ.
  const Eigen::Vector3d p_BQ = X_BG * p_GQ;
  const Vector3<symbolic::Polynomial> p_AQ =
      X_AB_multilinear.position + X_AB_multilinear.rotation * p_BQ;
  // Compute lhs=aᵀ*p_AQ + b
  const symbolic::Polynomial lhs = a_A.dot(p_AQ) + b;
  // Compute rhs=offset + separating_margin
  // Note that separating_margin is NOT an indeterminates.
  const symbolic::Polynomial rhs =
      offset + (separating_margin.has_value()
                    ? symbolic::Polynomial(
                          {{symbolic::Monomial(), separating_margin.value()}})
                    : symbolic::Polynomial(nullptr));
  if (plane_side == PlaneSide::kPositive) {
    return rational_forward_kin.ConvertMultilinearPolynomialToRationalFunction(
        lhs - rhs);
  } else {
    return rational_forward_kin.ConvertMultilinearPolynomialToRationalFunction(
        -rhs - lhs);
  }
}
}  // namespace

namespace {
void ImplementPolytopeGeometry(const Eigen::Ref<const Eigen::Matrix3Xd>& p_GV,
                               const math::RigidTransformd& X_BG, void* data) {
  // For a polytope (including box) to be on one side of the separating plane,
  // if we only require strict separation but don't explicitly constrain the
  // separating margin to be at least δ, then we impose the following
  // constraint:
  // aᵀ*p_AV+b ≥ 1 if plane_side = kPositive
  // aᵀ*p_AV+b ≤ -1 if plane_side = kNegative.
  //
  // If we require the separating margin to be at least δ, then we impose the
  // following constraint:
  // aᵀ*p_AV+b ≥ δ if plane_side = kPositive
  // aᵀ*p_AV+b ≤ -δ if plane_side = kNegative
  // and the constraint |a|≤1.
  auto* reify_data = static_cast<ReifyData*>(data);

  reify_data->rationals->reserve(p_GV.cols());
  // The position of the vertices V in the geometry frame G.
  const double offset = reify_data->separating_margin->has_value() ? 0 : 1.;
  for (int i = 0; i < p_GV.cols(); ++i) {
    reify_data->rationals->push_back(ComputePointOnPlaneSideRational(
        *(reify_data->a), *(reify_data->b), p_GV.col(i), X_BG,
        *(reify_data->X_AB_multilinear), offset,
        *(reify_data->separating_margin), reify_data->plane_side,
        *(reify_data->rational_forward_kin)));
  }
  if (reify_data->separating_margin->has_value()) {
    reify_data->unit_length_vector->emplace(*(reify_data->a));
  } else {
    reify_data->unit_length_vector->reset();
  }
}
}  // namespace
void CollisionGeometry::ImplementGeometry(const Box& box, void* data) {
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
  ImplementPolytopeGeometry(p_GV, X_BG_, data);
}

void CollisionGeometry::ImplementGeometry(const Convex& convex, void* data) {
  const Eigen::Matrix3Xd p_GV = GetVertices(convex);
  ImplementPolytopeGeometry(p_GV, X_BG_, data);
}

void CollisionGeometry::ImplementGeometry(const Sphere& sphere, void* data) {
  // If the sphere with radius r is on one side of the plane with a margin δ, it
  // is equivalent to the following condition
  // aᵀ*p_AS + b ≥ r + δ       if plane_side = kPositive   (1a)
  // aᵀ*p_AS + b ≤ -(r + δ)    if plane_side = kNegative   (1b)
  // |a| ≤ 1                                               (2)
  // where S is the center of the sphere.
  auto* reify_data = static_cast<ReifyData*>(data);

  reify_data->rationals->push_back(ComputePointOnPlaneSideRational(
      *(reify_data->a), *(reify_data->b), Eigen::Vector3d::Zero(), X_BG_,
      *(reify_data->X_AB_multilinear), sphere.radius(),
      *(reify_data->separating_margin), reify_data->plane_side,
      *(reify_data->rational_forward_kin)));
  reify_data->unit_length_vector->emplace(*(reify_data->a));
}

void CollisionGeometry::ImplementGeometry(const Capsule& capsule, void* data) {
  // If the capsule with radius r is on one side of the plane with a margin δ,
  // it is equivalent to the following condition
  // aᵀ*p_AS1 + b ≥ r + δ       if plane_side = kPositive   (1a)
  // aᵀ*p_AS2 + b ≥ r + δ       if plane_side = kPositive   (1a)
  // aᵀ*p_AS1 + b ≤ -(r + δ)    if plane_side = kNegative   (1b)
  // aᵀ*p_AS2 + b ≤ -(r + δ)    if plane_side = kNegative   (1b)
  // |a| ≤ 1                                               (2)
  // where S1 and S2 are the center of the two spheres on the two ends of the
  // capsule.
  auto* reify_data = static_cast<ReifyData*>(data);
  Eigen::Matrix<double, 3, 2> p_GS;
  p_GS.col(0) = Eigen::Vector3d(0, 0, capsule.length() / 2);
  p_GS.col(1) = Eigen::Vector3d(0, 0, -capsule.length() / 2);
  for (int i = 0; i < 2; ++i) {
    reify_data->rationals->push_back(ComputePointOnPlaneSideRational(
        *(reify_data->a), *(reify_data->b), p_GS.col(i), X_BG_,
        *(reify_data->X_AB_multilinear), capsule.radius(),
        *(reify_data->separating_margin), reify_data->plane_side,
        *(reify_data->rational_forward_kin)));
  }
  reify_data->unit_length_vector->emplace(*(reify_data->a));
}

void CollisionGeometry::ImplementGeometry(const Cylinder& cylinder,
                                          void* data) {
  // If we have a cylinder with radius r, to enforce the condition that an
  // inflated cylinder (whose radius is increased to r+δ, but the length is
  // unchanged) is on one side of the plane, we impose the following constraint:
  // aᵀ*p_AS1 + b ≥ r + δ if plane_side = kPositive    (1a)
  // aᵀ*p_AS2 + b ≥ r + δ if plane_side = kPositive    (1a)
  // aᵀ*p_AS1 + b ≤ -(r + δ) if plane_side = kNegative (1b)
  // aᵀ*p_AS2 + b ≤ -(r + δ) if plane_side = kNegative (1b)
  // |P*a| ≤ 1                                          (2)
  // where S1, S2 are the centers of the circle on the two ends of the cylinder.
  // P is matrix that projects a vector to the surface perpendicular to the
  // cylinder axis. If we denote the cylinder axis vector as n₃, and pick two
  // other unit-length vectors n₁, n₂, such that the matrix [n₁, n₂, n₃] is an
  // SO(3) matrix, then we know that |P*a| = |[n₁ᵀa, n₂ᵀa]|. If we denote
  // N= [n₁ᵀ; n₂ᵀ], we can impose |Na| <= 1
  //
  // Here we quickly derive equation 1a. If the cylinder with radius r+δ is on
  // the positive side of the plane, it is equivalent to all points on the rim
  // of the two end circles being on the positive side of the plane, namely
  // aᵀ(p_AS1 + (r+δ)*Nᵀ*t)+b≥0,
  // aᵀ(p_AS2 + (r+δ)*Nᵀ*t)+b≥0
  // ∀ t ∈ ℝ², |t|≤ 1;
  //
  // this is equivalent to
  // aᵀ*p_AS1 + b ≥ (r+δ)*|Na|
  // aᵀ*p_AS2 + b ≥ (r+δ)*|Na|
  // If we further constraint |Na| <= 1, then we have (1a)
  // aᵀ*p_AS1 + b ≥ r + δ
  // aᵀ*p_AS2 + b ≥ r + δ
  auto* reify_data = static_cast<ReifyData*>(data);

  Eigen::Matrix<double, 3, 2> p_GS;
  p_GS.col(0) = Eigen::Vector3d(0, 0, cylinder.length() / 2);
  p_GS.col(1) = Eigen::Vector3d(0, 0, -cylinder.length() / 2);
  for (int i = 0; i < 2; ++i) {
    reify_data->rationals->push_back(ComputePointOnPlaneSideRational(
        *(reify_data->a), *(reify_data->b), p_GS.col(i), X_BG_,
        *(reify_data->X_AB_multilinear), cylinder.radius(),
        *(reify_data->separating_margin), reify_data->plane_side,
        *(reify_data->rational_forward_kin)));
  }
  // We will impose the constraint |N*a|<=1. Note that a is expressed in frame
  // A, so we will also express N in the frame A. We start from N expressed in
  // the geometry frame G.
  Eigen::Matrix<double, 2, 3> N_G;
  N_G.row(0) = Eigen::Vector3d(1, 0, 0);
  N_G.row(1) = Eigen::Vector3d(0, 1, 0);
  // Now I need to convert N from frame G to frame A, we first convert it to
  // body frame A..
  Eigen::Matrix<double, 2, 3> N_B;
  N_B.row(0) = (X_BG_.rotation() * N_G.row(0).transpose()).transpose();
  N_B.row(1) = (X_BG_.rotation() * N_G.row(1).transpose()).transpose();
  // N_a *a_A is a rational function (because N_a is a matrix of rationals).
  Vector2<symbolic::RationalFunction> Na;
  for (int i = 0; i < 2; ++i) {
    Na(i) = reify_data->rational_forward_kin
                ->ConvertMultilinearPolynomialToRationalFunction(
                    (reify_data->X_AB_multilinear->rotation *
                     N_B.row(i).transpose())
                        .dot(*(reify_data->a)));
  }
  // N*a is a vector of rationals. Note that the denominator in these rationals
  // are the same. We denote N * a = p(s) / d(s), where p(s) is a vector of size
  // 2, d(s) is a scalar polynomial. By Schur's complement, |p(s) / d(s)| <= 1
  // if and ony if the following matrix
  // [d(s)² p(s)ᵀ]
  // [p(s)    I  ]
  // is always psd.
  DRAKE_DEMAND(Na(0).denominator().EqualTo(Na(1).denominator()));
  // To constrain that this matrix is always psd, we will introduce the new
  // indeterminates z with the condition zᵀz + 2p(s)ᵀz + d(s)² >= 0 on C*s<=d.
  // Notice that d(s)² might have high degree so I will need high-degree
  // Lagrangian multiplier.
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
