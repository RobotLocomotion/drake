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
      const std::optional<symbolic::Variable>& m_separating_margin,
      PlaneSide m_plane_side, GeometryId m_geometry_id,
      double m_polytope_chebyshev_radius_multiplier,
      std::vector<symbolic::RationalFunction>* m_rationals,
      std::optional<VectorX<symbolic::Polynomial>>* m_unit_length_vector)
      : a{&m_a},
        b{&m_b},
        X_AB_multilinear{&m_X_AB_multilinear},
        rational_forward_kin{&m_rational_forward_kin},
        separating_margin{&m_separating_margin},
        plane_side{m_plane_side},
        geometry_id{m_geometry_id},
        polytope_chebyshev_radius_multiplier{
            m_polytope_chebyshev_radius_multiplier},
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
  const GeometryId geometry_id;
  const double polytope_chebyshev_radius_multiplier;
  std::vector<symbolic::RationalFunction>* rationals;
  std::optional<VectorX<symbolic::Polynomial>>* unit_length_vector;
};

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

void ImplementPolytopeGeometry(const Eigen::Ref<const Eigen::Matrix3Xd>& p_GV,
                               const math::RigidTransformd& X_BG, void* data) {
  // For a polytope (including box) to be on one side of the separating plane
  // (and the other side geometry is also a polytope), we impose the following
  // constraint:
  // if plane_side = kPositive
  //   aᵀ*p_AV+b ≥ δ
  //   aᵀ*p_AC+b ≥ k*r
  // if plane_side = kNegative
  //   aᵀ*p_AV+b ≤ -δ
  //   aᵀ*p_AC+b ≤ -k*r
  // where C is the Chebyshev center of the polytope, and r is the distance
  // from C to the polytope boundary. k is a positive scalar between (0, 1),
  // defined in polytope_chebyshev_radius_multiplier_
  //
  // We impose the condition aᵀ*p_AC+b ≥ k*r (or ≤ -k*r) to rule out the
  // trivial solution a = 0, b=0, since the right hand side k*r (or -k*r) is
  // strictly non-zero.
  //
  // If separating_margin has value, then we also impose the constraint |a|≤1.
  auto* reify_data = static_cast<ReifyData*>(data);

  const double offset{0};
  const int num_rationals = p_GV.cols() + 1;
  reify_data->rationals->reserve(num_rationals);
  // The position of the vertices V in the geometry frame G.
  for (int i = 0; i < p_GV.cols(); ++i) {
    reify_data->rationals->push_back(ComputePointOnPlaneSideRational(
        *(reify_data->a), *(reify_data->b), p_GV.col(i), X_BG,
        *(reify_data->X_AB_multilinear), offset,
        *(reify_data->separating_margin), reify_data->plane_side,
        *(reify_data->rational_forward_kin)));
  }
  const HPolyhedron h_polyhedron{VPolytope(p_GV)};
  const Eigen::Vector3d p_GC = h_polyhedron.ChebyshevCenter();
  const double radius = ((h_polyhedron.b() - h_polyhedron.A() * p_GC).array() /
                         h_polyhedron.A().rowwise().norm().array())
                            .minCoeff();
  reify_data->rationals->push_back(ComputePointOnPlaneSideRational(
      *(reify_data->a), *(reify_data->b), p_GC, X_BG,
      *(reify_data->X_AB_multilinear),
      reify_data->polytope_chebyshev_radius_multiplier * radius, std::nullopt,
      reify_data->plane_side, *(reify_data->rational_forward_kin)));
  if (reify_data->separating_margin->has_value()) {
    reify_data->unit_length_vector->emplace(*(reify_data->a));
  } else {
    reify_data->unit_length_vector->reset();
  }
}

class OnPlaneSideReifier : public ShapeReifier {
 public:
  OnPlaneSideReifier(const Shape* geometry, math::RigidTransformd X_BG,
                     GeometryId geometry_id,
                     double polytope_chebyshev_radius_multiplier)
      : geometry_{geometry},
        X_BG_{std::move(X_BG)},
        geometry_id_{geometry_id},
        polytope_chebyshev_radius_multiplier_{
            polytope_chebyshev_radius_multiplier} {}

  void ProcessData(
      const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
      const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
          X_AB_multilinear,
      const multibody::RationalForwardKinematics& rational_forward_kin,
      const std::optional<symbolic::Variable>& separating_margin,
      PlaneSide plane_side, std::vector<symbolic::RationalFunction>* rationals,
      std::optional<VectorX<symbolic::Polynomial>>* unit_length_vector) {
    ReifyData data(a, b, X_AB_multilinear, rational_forward_kin,
                   separating_margin, plane_side, geometry_id_,
                   polytope_chebyshev_radius_multiplier_, rationals,
                   unit_length_vector);
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
    ImplementPolytopeGeometry(p_GV, X_BG_, data);
  }

  void ImplementGeometry(const Convex& convex, void* data) {
    const Eigen::Matrix3Xd p_GV = GetVertices(convex);
    ImplementPolytopeGeometry(p_GV, X_BG_, data);
  }

  void ImplementGeometry(const Sphere& sphere, void* data) {
    // If the sphere with radius r is on one side of the plane with a margin δ,
    // it is equivalent to the following condition
    // aᵀ*p_AS + b ≥ r + δ       if plane_side = kPositive   (1a)
    // aᵀ*p_AS + b ≤ -(r + δ)    if plane_side = kNegative   (1b)
    // |a| ≤ 1 (2)
    // where S is the center of the sphere.
    auto* reify_data = static_cast<ReifyData*>(data);

    reify_data->rationals->push_back(ComputePointOnPlaneSideRational(
        *(reify_data->a), *(reify_data->b), Eigen::Vector3d::Zero(), X_BG_,
        *(reify_data->X_AB_multilinear), sphere.radius(),
        *(reify_data->separating_margin), reify_data->plane_side,
        *(reify_data->rational_forward_kin)));
    reify_data->unit_length_vector->emplace(*(reify_data->a));
  }

  void ImplementGeometry(const Capsule& capsule, void* data) {
    // If the capsule with radius r is on one side of the plane with a margin δ,
    // it is equivalent to the following condition
    // aᵀ*p_AS1 + b ≥ r + δ       if plane_side = kPositive   (1a)
    // aᵀ*p_AS2 + b ≥ r + δ       if plane_side = kPositive   (2a)
    // aᵀ*p_AS1 + b ≤ -(r + δ)    if plane_side = kNegative   (1b)
    // aᵀ*p_AS2 + b ≤ -(r + δ)    if plane_side = kNegative   (2b)
    // |a| ≤ 1                                                 (3)
    // where S1 and S2 are the center of the two spheres on the two ends of the
    // capsule.
    auto* reify_data = static_cast<ReifyData*>(data);
    Eigen::Matrix<double, 3, 2> p_GS;
    p_GS.col(0) = Eigen::Vector3d(0, 0, capsule.length() / 2);
    p_GS.col(1) = Eigen::Vector3d(0, 0, -capsule.length() / 2);
    reify_data->rationals->reserve(reify_data->rationals->size() + 2);
    for (int i = 0; i < 2; ++i) {
      reify_data->rationals->push_back(ComputePointOnPlaneSideRational(
          *(reify_data->a), *(reify_data->b), p_GS.col(i), X_BG_,
          *(reify_data->X_AB_multilinear), capsule.radius(),
          *(reify_data->separating_margin), reify_data->plane_side,
          *(reify_data->rational_forward_kin)));
    }
    reify_data->unit_length_vector->emplace(*(reify_data->a));
  }

  const Shape* geometry_;
  math::RigidTransformd X_BG_;
  GeometryId geometry_id_;
  double polytope_chebyshev_radius_multiplier_;
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

class NumRationalsPerPlaneSideReifier : public ShapeReifier {
 public:
  explicit NumRationalsPerPlaneSideReifier(const Shape* shape)
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
    *num = 9;
  }

  void ImplementGeometry(const Convex& convex, void* data) {
    auto* num = static_cast<int*>(data);
    const Eigen::Matrix3Xd p_GV = GetVertices(convex);
    *num = p_GV.cols() + 1;
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

  const Shape* shape_;
  Eigen::Vector3d a_G_;
  double b_G_;
};
}  // namespace

void CollisionGeometry::OnPlaneSide(
    const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
    const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
        X_AB_multilinear,
    const multibody::RationalForwardKinematics& rational_forward_kin,
    const std::optional<symbolic::Variable>& separating_margin,
    PlaneSide plane_side, std::vector<symbolic::RationalFunction>* rationals,
    std::optional<VectorX<symbolic::Polynomial>>* unit_length_vector) const {
  OnPlaneSideReifier reifier(geometry_, X_BG_, id_,
                             polytope_chebyshev_radius_multiplier_);
  reifier.ProcessData(a, b, X_AB_multilinear, rational_forward_kin,
                      separating_margin, plane_side, rationals,
                      unit_length_vector);
}

GeometryType CollisionGeometry::type() const {
  GeometryTypeReifier reifier(geometry_);
  return reifier.ProcessData();
}

int CollisionGeometry::num_rationals_per_side() const {
  NumRationalsPerPlaneSideReifier reifier(geometry_);
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
