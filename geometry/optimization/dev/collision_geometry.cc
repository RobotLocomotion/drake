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
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
