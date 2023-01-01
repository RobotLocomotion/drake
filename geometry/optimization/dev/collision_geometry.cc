#include "drake/geometry/optimization/dev/collision_geometry.h"

#include <utility>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"

namespace drake {
namespace geometry {
namespace optimization {

PlaneSide OtherSide(PlaneSide plane_side) {
  return plane_side == PlaneSide::kPositive ? PlaneSide::kNegative
                                            : PlaneSide::kPositive;
}

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
      PlaneSide m_plane_side, GeometryId m_geometry_id,
      const VectorX<symbolic::Variable>& m_y_slack,
      std::vector<symbolic::RationalFunction>* m_rationals_for_points,
      std::vector<symbolic::RationalFunction>* m_rationals_for_matrix_sos)
      : a{&m_a},
        b{&m_b},
        X_AB_multilinear{&m_X_AB_multilinear},
        rational_forward_kin{&m_rational_forward_kin},
        plane_side{m_plane_side},
        geometry_id{m_geometry_id},
        y_slack{&m_y_slack},
        rationals_for_points{m_rationals_for_points},
        rationals_for_matrix_sos{m_rationals_for_matrix_sos} {}

  // To avoid copying objects (which might be expensive), I store the
  // non-primitive-type objects with pointers.
  const Vector3<symbolic::Polynomial>* a;
  const symbolic::Polynomial* b;
  const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>*
      X_AB_multilinear;
  const multibody::RationalForwardKinematics* rational_forward_kin;
  const PlaneSide plane_side;
  const GeometryId geometry_id;
  const VectorX<symbolic::Variable>* y_slack;
  std::vector<symbolic::RationalFunction>* rationals_for_points;
  std::vector<symbolic::RationalFunction>* rationals_for_matrix_sos;
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
                             const math::RigidTransformd& X_BG, void* data) {
  // As part of the conditions that a geometry is on one side of the separating
  // plane, we impose the following constraint:
  // if plane_side = kPositive
  //   aᵀ*p_AV+b ≥ 1
  // if plane_side = kNegative
  //   aᵀ*p_AV+b ≤ -1
  auto* reify_data = static_cast<ReifyData*>(data);

  const double offset{1};
  const int num_rationals_for_points = p_GV.cols();
  reify_data->rationals_for_points->reserve(num_rationals_for_points);
  // The position of the vertices V in the geometry frame G.
  for (int i = 0; i < p_GV.cols(); ++i) {
    reify_data->rationals_for_points->push_back(ComputePointOnPlaneSideRational(
        *(reify_data->a), *(reify_data->b), p_GV.col(i), X_BG,
        *(reify_data->X_AB_multilinear), offset, reify_data->plane_side,
        *(reify_data->rational_forward_kin)));
  }
}

void ImplementSpherePsdMatRational(const Eigen::Vector3d& p_GS,
                                   const math::RigidTransformd& X_BG,
                                   double radius, void* data) {
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
  auto* reify_data = static_cast<ReifyData*>(data);

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
  reify_data->rationals_for_matrix_sos->emplace_back(
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
      std::vector<symbolic::RationalFunction>* rationals_for_points,
      std::vector<symbolic::RationalFunction>* rationals_for_matrix_sos) {
    ReifyData data(a, b, X_AB_multilinear, rational_forward_kin, plane_side,
                   geometry_id_, y_slack, rationals_for_points,
                   rationals_for_matrix_sos);
    geometry_->Reify(this, &data);
  }

 private:
  // Implements supported shapes for the ShapeReifier interface.
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box& box, void* data) {
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
    ImplementPointRationals(p_GV, X_BG_, data);
  }

  void ImplementGeometry(const Convex& convex, void* data) {
    const Eigen::Matrix3Xd p_GV = GetVertices(convex);
    ImplementPointRationals(p_GV, X_BG_, data);
  }

  void ImplementGeometry(const Sphere& sphere, void* data) {
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
                                  sphere.radius(), data);
    // Now add the rational constraint
    // aᵀ*p_AS + b ≥ 1          if plane_side = kPositive   (1b)
    // aᵀ*p_AS + b ≤ -1         if plane_side = kNegative   (2b)
    ImplementPointRationals(Eigen::Vector3d::Zero(), X_BG_, data);
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
    ImplementSpherePsdMatRational(Eigen::Vector3d(0, 0, capsule.length() / 2),
                                  X_BG_, capsule.radius(), data);
    ImplementSpherePsdMatRational(Eigen::Vector3d(0, 0, -capsule.length() / 2),
                                  X_BG_, capsule.radius(), data);
    // aᵀ*p_AO + b ≥ 1          if plane_side = kPositive
    // aᵀ*p_AO + b ≤ -1         if plane_side = kNegative
    ImplementPointRationals(Eigen::Vector3d::Zero(), X_BG_, data);
  }

  const Shape* geometry_;
  math::RigidTransformd X_BG_;
  GeometryId geometry_id_;
};

class GeometryTypeReifier : public ShapeReifier {
 public:
  explicit GeometryTypeReifier(const geometry::Shape* shape) : shape_{shape} {}

  GeometryType ProcessData() {
    GeometryType type;
    shape_->Reify(this, &type);
    return type;
  }

 private:
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Box&, void* data) {
    auto* type = static_cast<GeometryType*>(data);
    *type = GeometryType::kPolytope;
  }

  void ImplementGeometry(const Convex&, void* data) {
    auto* type = static_cast<GeometryType*>(data);
    *type = GeometryType::kPolytope;
  }

  void ImplementGeometry(const Sphere&, void* data) {
    auto* type = static_cast<GeometryType*>(data);
    *type = GeometryType::kSphere;
  }

  void ImplementGeometry(const Capsule&, void* data) {
    auto* type = static_cast<GeometryType*>(data);
    *type = GeometryType::kCapsule;
  }

  void ImplementGeometry(const Cylinder&, void* data) {
    auto* type = static_cast<GeometryType*>(data);
    *type = GeometryType::kCylinder;
  }

  const Shape* shape_;
};

class NumRationalsForPointsReifier : public ShapeReifier {
 public:
  explicit NumRationalsForPointsReifier(const Shape* shape) : shape_{shape} {}

  int ProcessData() {
    int ret;
    shape_->Reify(this, &ret);
    return ret;
  }

 private:
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box&, void* data) {
    auto* num = static_cast<int*>(data);
    *num = 8;
  }

  void ImplementGeometry(const Convex& convex, void* data) {
    auto* num = static_cast<int*>(data);
    const Eigen::Matrix3Xd p_GV = GetVertices(convex);
    *num = p_GV.cols();
  }

  void ImplementGeometry(const Sphere&, void* data) {
    auto* num = static_cast<int*>(data);
    *num = 1;
  }

  void ImplementGeometry(const Capsule&, void* data) {
    auto* num = static_cast<int*>(data);
    *num = 1;
  }

  const Shape* shape_;
};

class NumRationalsForMatrixSosReifier : public ShapeReifier {
 public:
  explicit NumRationalsForMatrixSosReifier(const Shape* shape)
      : shape_{shape} {}

  int ProcessData() {
    int ret;
    shape_->Reify(this, &ret);
    return ret;
  }

 private:
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box&, void* data) {
    auto* num = static_cast<int*>(data);
    *num = 0;
  }

  void ImplementGeometry(const Convex&, void* data) {
    auto* num = static_cast<int*>(data);
    *num = 0;
  }

  void ImplementGeometry(const Sphere&, void* data) {
    auto* num = static_cast<int*>(data);
    *num = 1;
  }

  void ImplementGeometry(const Capsule&, void* data) {
    auto* num = static_cast<int*>(data);
    *num = 2;
  }

  const Shape* shape_;
};

/**
 Compute the signed distance from a collision geometry to the halfspace { p_GQ |
 a_Gᵀ*p_GQ+b_G >= 0}, where a_G is in the collision geometry frame.
 */
class DistanceToHalfspaceReifier : public ShapeReifier {
 public:
  explicit DistanceToHalfspaceReifier(const Shape* shape, Eigen::Vector3d a_G,
                                      double b_G)
      : shape_{shape}, a_G_{std::move(a_G)}, b_G_{b_G} {}

  double ProcessData() {
    double ret;
    shape_->Reify(this, &ret);
    return ret;
  }

 private:
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box& box, void* data) {
    double* distance = static_cast<double*>(data);
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
    *distance = ((a_G_.transpose() * p_GV).minCoeff() + b_G_) / (a_G_.norm());
  }

  void ImplementGeometry(const Convex& convex, void* data) {
    double* distance = static_cast<double*>(data);
    *distance = ((a_G_.transpose() * GetVertices(convex)).minCoeff() + b_G_) /
                (a_G_.norm());
  }

  void ImplementGeometry(const Sphere& sphere, void* data) {
    double* distance = static_cast<double*>(data);
    *distance = b_G_ / (a_G_.norm()) - sphere.radius();
  }

  void ImplementGeometry(const Capsule& capsule, void* data) {
    double* distance = static_cast<double*>(data);
    Eigen::Matrix<double, 3, 2> p_GS;
    p_GS.col(0) << 0, 0, capsule.length() / 2;
    p_GS.col(1) << 0, 0, -capsule.length() / 2;
    *distance = ((a_G_.transpose() * p_GS).minCoeff() + b_G_) / (a_G_.norm()) -
                capsule.radius();
  }

  void ImplementGeometry(const Cylinder& cylinder, void* data) {
    // The distance from a point on the top rim [r*cosθ, r*sinθ, h/2] to the
    // face {x | a.dot(x)+b=0} is (a(0)*r*cosθ + a(1)*r*sinθ + a(2)*h/2 + b) /
    // |a|. Taking the minimum over θ, we get the distance from the top rim to
    // the face as (-r * |[a(0), a(1)]| + a(2)*h/2 + b) / |a|. Similarly, the
    // distance from the bottom rim [r*cosθ, r*sinθ, -h/2] to the face {x |
    // a.dot(x)+b=0} is
    // (-r * |[a(0), a(1)]| - a(2)*h/2 + b) / |a|.
    double* distance = static_cast<double*>(data);
    *distance =
        (-cylinder.radius() * a_G_.head<2>().norm() +
         (a_G_(2) >= 0 ? -a_G_(2) : a_G_(2)) * cylinder.length() / 2 + b_G_) /
        a_G_.norm();
  }

  const Shape* shape_;
  Eigen::Vector3d a_G_;
  double b_G_;
};

class YSlackSizeReifier : public ShapeReifier {
 public:
  explicit YSlackSizeReifier(const Shape* shape) : shape_{shape} {}

  double ProcessData() {
    int ret;
    shape_->Reify(this, &ret);
    return ret;
  }

 private:
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box&, void* data) {
    int* y_slack_size = static_cast<int*>(data);
    *y_slack_size = 0;
  }

  void ImplementGeometry(const Convex&, void* data) {
    int* y_slack_size = static_cast<int*>(data);
    *y_slack_size = 0;
  }

  void ImplementGeometry(const Sphere&, void* data) {
    int* y_slack_size = static_cast<int*>(data);
    *y_slack_size = 3;
  }

  void ImplementGeometry(const Capsule&, void* data) {
    int* y_slack_size = static_cast<int*>(data);
    *y_slack_size = 3;
  }

  const Shape* shape_;
};
}  // namespace

void CollisionGeometry::OnPlaneSide(
    const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
    const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
        X_AB_multilinear,
    const multibody::RationalForwardKinematics& rational_forward_kin,
    PlaneSide plane_side, const VectorX<symbolic::Variable>& y_slack,
    std::vector<symbolic::RationalFunction>* rationals_for_points,
    std::vector<symbolic::RationalFunction>* rationals_for_matrix_sos) const {
  OnPlaneSideReifier reifier(geometry_, X_BG_, id_);
  reifier.ProcessData(a, b, X_AB_multilinear, rational_forward_kin, plane_side,
                      y_slack, rationals_for_points, rationals_for_matrix_sos);
}

GeometryType CollisionGeometry::type() const {
  GeometryTypeReifier reifier(geometry_);
  return reifier.ProcessData();
}

int CollisionGeometry::num_rationals_for_points() const {
  NumRationalsForPointsReifier reifier(geometry_);
  return reifier.ProcessData();
}

int CollisionGeometry::num_rationals_for_matrix_sos() const {
  NumRationalsForMatrixSosReifier reifier(geometry_);
  return reifier.ProcessData();
}

int CollisionGeometry::y_slack_size() const {
  YSlackSizeReifier reifier(geometry_);
  return reifier.ProcessData();
}

double DistanceToHalfspace(const CollisionGeometry& collision_geometry,
                           const Eigen::Vector3d& a, double b,
                           multibody::BodyIndex expressed_body,
                           PlaneSide plane_side,
                           const multibody::MultibodyPlant<double>& plant,
                           const systems::Context<double>& plant_context) {
  // Transforms the halfspace from expressed frame (E) to the collision_geometry
  // geometry frame (G). First compute the pose of expressed frame (E) to the
  // collision geometry body frame (B).
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
  Eigen::Vector3d a_G;
  double b_G;
  a_G = X_GE.rotation() * a;
  b_G = b - a_G.dot(X_GE.translation());
  if (plane_side == PlaneSide::kNegative) {
    a_G = -a_G;
    b_G = -b_G;
  }

  DistanceToHalfspaceReifier reifier(&(collision_geometry.geometry()), a_G,
                                     b_G);
  return reifier.ProcessData();
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
