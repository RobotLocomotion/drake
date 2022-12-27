#include "drake/geometry/optimization/dev/collision_geometry.h"

#include <memory>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/optimization/dev/test/c_iris_test_utilities.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"

namespace drake {
namespace geometry {
namespace optimization {

void SetupPlane(const Eigen::Ref<const VectorX<symbolic::Variable>>& s,
                Vector3<symbolic::Polynomial>* a, symbolic::Polynomial* b) {
  // Set each entry in a and b as an affine polynomial of s.
  Matrix3X<symbolic::Variable> a_coeff(3, s.rows());
  Vector3<symbolic::Variable> a_constant;
  const symbolic::Variables s_set(s);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < s.rows(); ++j) {
      a_coeff(i, j) = symbolic::Variable(fmt::format("a_coeff({}, {})", i, j));
    }
    a_constant(i) = symbolic::Variable(fmt::format("a_constant({})", i));
    (*a)(i) = symbolic::Polynomial(
        a_coeff.row(i).dot(s.cast<symbolic::Expression>()) + a_constant(i),
        s_set);
  }
  VectorX<symbolic::Variable> b_coeff(s.rows());
  const symbolic::Variable b_constant("b_constant");
  for (int i = 0; i < s.rows(); ++i) {
    b_coeff(i) = symbolic::Variable(fmt::format("b_coeff({})", i));
  }
  *b = symbolic::Polynomial(
      b_coeff.dot(s.cast<symbolic::Expression>()) + b_constant, s_set);
}

class CollisionGeometryTest : public CIrisToyRobotTest {
 public:
  CollisionGeometryTest()
      : rational_forward_kin_{plant_},
        diagram_context_{diagram_->CreateDefaultContext()} {
    SetupPlane(rational_forward_kin_.s(), &a_, &b_);
    plant_context_ = &(
        diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get()));
    scene_graph_context_ = &(diagram_->GetMutableSubsystemContext(
        *scene_graph_, diagram_context_.get()));
  }

  Eigen::VectorXd SetQ(const Eigen::Vector3d& q_star,
                       const Eigen::Vector3d& q_val) {
    DRAKE_DEMAND(
        (q_val.array() <= plant_->GetPositionUpperLimits().array()).all());
    DRAKE_DEMAND(
        (q_val.array() >= plant_->GetPositionLowerLimits().array()).all());
    const Eigen::VectorXd s_val =
        rational_forward_kin_.ComputeSValue(q_val, q_star);
    plant_->SetPositions(plant_context_, q_val);
    return s_val;
  }

 protected:
  multibody::RationalForwardKinematics rational_forward_kin_;
  Vector3<symbolic::Polynomial> a_;
  symbolic::Polynomial b_;
  std::unique_ptr<systems::Context<double>> diagram_context_;
  systems::Context<double>* plant_context_;
  systems::Context<double>* scene_graph_context_;

  Eigen::Vector3d q_val_;
};

void CheckRationalExpression(const symbolic::RationalFunction& rational,
                             const symbolic::Environment& env,
                             const symbolic::Expression& expr_expected) {
  const symbolic::Expression expr =
      rational.numerator().EvaluatePartial(env).ToExpression() /
      rational.denominator().Evaluate(env);
  EXPECT_PRED3(symbolic::test::PolynomialEqual, symbolic::Polynomial(expr),
               symbolic::Polynomial(expr_expected), 1E-7);
}

TEST_F(CollisionGeometryTest, Box) {
  // Test CollisionGeometry constructed from a box.
  const auto& model_inspector = scene_graph_->model_inspector();
  const multibody::BodyIndex geometry_body = body_indices_[0];
  CollisionGeometry box(&model_inspector.GetShape(body0_box_), geometry_body,
                        body0_box_, model_inspector.GetPoseInFrame(body0_box_));
  EXPECT_EQ(box.type(), GeometryType::kPolytope);

  Eigen::Vector3d q_star(0., 0., 0.);
  const multibody::BodyIndex expressed_body = body_indices_[1];
  const auto X_AB_multilinear =
      rational_forward_kin_.CalcBodyPoseAsMultilinearPolynomial(
          q_star, geometry_body, expressed_body);

  std::vector<symbolic::RationalFunction> rationals;
  std::optional<VectorX<symbolic::Polynomial>> unit_length_vector;
  // Positive side, separating_margin is empty.
  box.OnPlaneSide(a_, b_, X_AB_multilinear, rational_forward_kin_, std::nullopt,
                  PlaneSide::kPositive, &rationals, &unit_length_vector);
  EXPECT_FALSE(unit_length_vector.has_value());
  EXPECT_EQ(box.num_rationals_per_side(), 9);
  EXPECT_EQ(rationals.size(), 9);

  // The order of the vertices should be the same in collision_geometry.cc
  Eigen::Matrix<double, 3, 8> p_GV;
  // clang-format off
  p_GV << 1, 1, 1, 1, -1, -1, -1, -1,
          1, 1, -1, -1, 1, 1, -1, -1,
          1, -1, 1, -1, 1, -1, 1, -1;
  // clang-format on
  auto box_geometry = static_cast<const Box&>(box.geometry());
  p_GV.row(0) *= box_geometry.width() / 2;
  p_GV.row(1) *= box_geometry.depth() / 2;
  p_GV.row(2) *= box_geometry.height() / 2;
  // C is the Chebyshev center. The Chebyshev center of the box is not unique,
  // we choose the one computed from HPolyhedron.
  const HPolyhedron box_polyhedron{VPolytope(p_GV)};
  const Eigen::Vector3d p_GC = box_polyhedron.ChebyshevCenter();
  const double radius =
      std::min(std::min(box_geometry.width(), box_geometry.depth()),
               box_geometry.height()) /
      2;

  const Eigen::Matrix<double, 3, 8> p_BV = box.X_BG() * p_GV;
  const Eigen::Vector3d p_BC = box.X_BG() * p_GC;

  // Evaluate the rationals, and compare the evaluation result with a.dot(p_AV)
  // + b
  Eigen::Vector3d q_val(0.2, -0.1, 0.5);
  const Eigen::VectorXd s_val = SetQ(q_star, q_val);

  symbolic::Environment env;
  env.insert(rational_forward_kin_.s(), s_val);
  Eigen::Matrix<double, 3, 8> p_AV;
  plant_->CalcPointsPositions(
      *plant_context_, plant_->get_body(geometry_body).body_frame(), p_BV,
      plant_->get_body(expressed_body).body_frame(), &p_AV);
  Eigen::Vector3d p_AC;
  plant_->CalcPointsPositions(
      *plant_context_, plant_->get_body(geometry_body).body_frame(), p_BC,
      plant_->get_body(expressed_body).body_frame(), &p_AC);
  Vector3<symbolic::Expression> a_expr;
  for (int j = 0; j < 3; ++j) {
    a_expr(j) = a_(j).EvaluatePartial(env).ToExpression();
  }
  const symbolic::Expression b_expr = b_.EvaluatePartial(env).ToExpression();
  for (int i = 0; i < 8; ++i) {
    const symbolic::Expression expr_expected = a_expr.dot(p_AV.col(i)) + b_expr;
    CheckRationalExpression(rationals[i], env, expr_expected);
  }
  CheckRationalExpression(rationals.back(), env,
                          a_expr.dot(p_AC) + b_expr - 0.5 * radius);

  // Negative side, with separating margin.
  rationals.clear();
  symbolic::Variable separating_margin{"delta"};
  box.OnPlaneSide(a_, b_, X_AB_multilinear, rational_forward_kin_,
                  separating_margin, PlaneSide::kNegative, &rationals,
                  &unit_length_vector);
  EXPECT_TRUE(unit_length_vector.has_value());
  EXPECT_EQ(unit_length_vector->rows(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ((*unit_length_vector)(i), a_(i));
  }
  EXPECT_EQ(rationals.size(), 9);
  EXPECT_EQ(box.num_rationals_per_side(), 9);
  for (int i = 0; i < 8; ++i) {
    const symbolic::Expression expr_expected =
        -separating_margin - a_expr.dot(p_AV.col(i)) - b_expr;
    CheckRationalExpression(rationals[i], env, expr_expected);
    EXPECT_TRUE(rationals[i].numerator().decision_variables().include(
        separating_margin));
  }
  CheckRationalExpression(rationals.back(), env,
                          -0.5 * radius - a_expr.dot(p_AC) - b_expr);
  EXPECT_FALSE(rationals.back().numerator().decision_variables().include(
      separating_margin));
}

TEST_F(CollisionGeometryTest, Convex) {
  // Test CollisionGeometry constructed from a convex object.
  const auto& model_inspector = scene_graph_->model_inspector();
  const multibody::BodyIndex geometry_body = body_indices_[1];
  CollisionGeometry convex(&model_inspector.GetShape(body1_convex_),
                           geometry_body, body1_convex_,
                           model_inspector.GetPoseInFrame(body1_convex_));
  EXPECT_EQ(convex.type(), GeometryType::kPolytope);

  const multibody::BodyIndex expressed_body = body_indices_[3];
  const Eigen::Vector3d q_star(0, 0, 0);
  const auto X_AB_multilinear =
      rational_forward_kin_.CalcBodyPoseAsMultilinearPolynomial(
          q_star, geometry_body, expressed_body);

  std::vector<symbolic::RationalFunction> rationals;
  std::optional<VectorX<symbolic::Polynomial>> unit_length_vector;

  auto query_object =
      scene_graph_->get_query_output_port().Eval<QueryObject<double>>(
          *scene_graph_context_);

  const VPolytope polytope(query_object, body1_convex_,
                           model_inspector.GetFrameId(body1_convex_));
  // Compute the H-polyhedron in the geometry frame.
  const HPolyhedron h_poly(
      VPolytope(convex.X_BG().inverse() * polytope.vertices()));

  const Eigen::Vector3d q_val(0.2, -0.1, 0.4);
  const Eigen::VectorXd s_val = SetQ(q_star, q_val);

  // The polytope reference frame is already set to the body, so I don't need to
  // multiply X_BG() here.
  const Eigen::Matrix3Xd p_BV = polytope.vertices();
  Eigen::Matrix3Xd p_AV(3, polytope.vertices().cols());
  plant_->CalcPointsPositions(
      *plant_context_, plant_->get_body(geometry_body).body_frame(), p_BV,
      plant_->get_body(expressed_body).body_frame(), &p_AV);
  const Eigen::Vector3d p_GC = h_poly.ChebyshevCenter();
  const Eigen::Vector3d p_BC = convex.X_BG() * p_GC;
  Eigen::Vector3d p_AC;
  plant_->CalcPointsPositions(
      *plant_context_, plant_->get_body(geometry_body).body_frame(), p_BC,
      plant_->get_body(expressed_body).body_frame(), &p_AC);
  const double radius = ((h_poly.b() - h_poly.A() * p_GC).array() /
                         h_poly.A().rowwise().norm().array())
                            .minCoeff();

  // negative side, no separating margin.
  convex.OnPlaneSide(a_, b_, X_AB_multilinear, rational_forward_kin_,
                     std::nullopt, PlaneSide::kNegative, &rationals,
                     &unit_length_vector);
  EXPECT_FALSE(unit_length_vector.has_value());
  EXPECT_EQ(rationals.size(), polytope.vertices().cols() + 1);
  EXPECT_EQ(convex.num_rationals_per_side(), polytope.vertices().cols() + 1);
  symbolic::Environment env;
  env.insert(rational_forward_kin_.s(), s_val);
  Vector3<symbolic::Expression> a_expr;
  for (int j = 0; j < 3; ++j) {
    a_expr(j) = a_(j).EvaluatePartial(env).ToExpression();
  }
  const symbolic::Expression b_expr = b_.EvaluatePartial(env).ToExpression();
  for (int i = 0; i < polytope.vertices().cols(); ++i) {
    const symbolic::Expression expr_expected =
        -a_expr.dot(p_AV.col(i)) - b_expr;
    CheckRationalExpression(rationals[i], env, expr_expected);
  }
  CheckRationalExpression(rationals.back(), env,
                          -0.5 * radius - a_expr.dot(p_AC) - b_expr);

  // Positive side, with separating margin.
  const symbolic::Variable separating_margin("delta");
  // Note that here I didn't clear rationals, so that I can test the new
  // rationals are appended to the existing ones.
  convex.OnPlaneSide(a_, b_, X_AB_multilinear, rational_forward_kin_,
                     separating_margin, PlaneSide::kPositive, &rationals,
                     &unit_length_vector);
  EXPECT_TRUE(unit_length_vector.has_value());
  EXPECT_EQ(unit_length_vector->rows(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ((*unit_length_vector)(i), a_(i));
  }
  // The new rationals are appended to the existing ones.
  EXPECT_EQ(rationals.size(), 2 * polytope.vertices().cols() + 2);
  EXPECT_EQ(convex.num_rationals_per_side(), 1 + polytope.vertices().cols());
  for (int i = 0; i < polytope.vertices().cols(); ++i) {
    const symbolic::Expression expr_expected =
        a_expr.dot(p_AV.col(i)) + b_expr - separating_margin;
    CheckRationalExpression(rationals[i + polytope.vertices().cols() + 1], env,
                            expr_expected);
    EXPECT_TRUE(rationals[i + polytope.vertices().cols() + 1]
                    .numerator()
                    .decision_variables()
                    .include(separating_margin));
  }
  CheckRationalExpression(rationals.back(), env,
                          a_expr.dot(p_AC) + b_expr - 0.5 * radius);
}

TEST_F(CollisionGeometryTest, Sphere) {
  // Test CollisionGeometry constructed from a sphere object.
  const auto& model_inspector = scene_graph_->model_inspector();
  CollisionGeometry sphere(&model_inspector.GetShape(world_sphere_),
                           plant_->world_body().index(), world_sphere_,
                           model_inspector.GetPoseInFrame(world_sphere_));
  EXPECT_EQ(sphere.type(), GeometryType::kSphere);
  EXPECT_EQ(sphere.num_rationals_per_side(), 1);

  const Eigen::Vector3d q_star(0., 0., 0.);
  const multibody::BodyIndex expressed_body = body_indices_[3];
  const auto X_AB_multilinear =
      rational_forward_kin_.CalcBodyPoseAsMultilinearPolynomial(
          q_star, plant_->world_body().index(), expressed_body);

  const Eigen::Vector3d q_val(0.2, -0.1, 0.4);
  const Eigen::Vector3d s_val = SetQ(q_star, q_val);
  symbolic::Environment env;
  env.insert(rational_forward_kin_.s(), s_val);
  Vector3<symbolic::Expression> a_expr;
  for (int j = 0; j < 3; ++j) {
    a_expr(j) = a_(j).EvaluatePartial(env).ToExpression();
  }
  const symbolic::Expression b_expr = b_.EvaluatePartial(env).ToExpression();

  // Negative side, no margin.
  std::vector<symbolic::RationalFunction> rationals;
  std::optional<VectorX<symbolic::Polynomial>> unit_length_vector;
  sphere.OnPlaneSide(a_, b_, X_AB_multilinear, rational_forward_kin_,
                     std::nullopt, PlaneSide::kNegative, &rationals,
                     &unit_length_vector);
  EXPECT_TRUE(unit_length_vector.has_value());
  EXPECT_EQ(unit_length_vector->rows(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ((*unit_length_vector)(i), a_(i));
  }
  EXPECT_EQ(rationals.size(), 1);
  const Eigen::Vector3d p_BS = sphere.X_BG().translation();
  Eigen::Vector3d p_AS;
  plant_->CalcPointsPositions(*plant_context_, plant_->world_frame(), p_BS,
                              plant_->get_body(expressed_body).body_frame(),
                              &p_AS);
  const double radius =
      static_cast<const Sphere&>(model_inspector.GetShape(world_sphere_))
          .radius();
  symbolic::Expression expr_expected = -radius - a_expr.dot(p_AS) - b_expr;
  CheckRationalExpression(rationals[0], env, expr_expected);

  // Positive side, with margin.
  rationals.clear();
  const symbolic::Variable separating_margin("delta");
  sphere.OnPlaneSide(a_, b_, X_AB_multilinear, rational_forward_kin_,
                     separating_margin, PlaneSide::kPositive, &rationals,
                     &unit_length_vector);
  EXPECT_TRUE(unit_length_vector.has_value());
  EXPECT_EQ(unit_length_vector->rows(), 3);
  expr_expected = a_expr.dot(p_AS) + b_expr - radius - separating_margin;
  CheckRationalExpression(rationals[0], env, expr_expected);
}

TEST_F(CollisionGeometryTest, Capsule) {
  // Test CollisionGeometry constructed from a capsule object.
  const auto& model_inspector = scene_graph_->model_inspector();
  const multibody::BodyIndex geometry_body = body_indices_[2];
  CollisionGeometry capsule(&model_inspector.GetShape(body2_capsule_),
                            geometry_body, body2_capsule_,
                            model_inspector.GetPoseInFrame(body2_capsule_));
  EXPECT_EQ(capsule.type(), GeometryType::kCapsule);
  EXPECT_EQ(capsule.num_rationals_per_side(), 2);

  const Eigen::Vector3d q_star(0., 0., 0.);
  const multibody::BodyIndex expressed_body = body_indices_[0];
  const auto X_AB_multilinear =
      rational_forward_kin_.CalcBodyPoseAsMultilinearPolynomial(
          q_star, geometry_body, expressed_body);

  const Eigen::Vector3d q_val(0.2, -0.1, 0.4);
  const Eigen::Vector3d s_val = SetQ(q_star, q_val);
  symbolic::Environment env;
  env.insert(rational_forward_kin_.s(), s_val);
  Vector3<symbolic::Expression> a_expr;
  for (int j = 0; j < 3; ++j) {
    a_expr(j) = a_(j).EvaluatePartial(env).ToExpression();
  }
  const symbolic::Expression b_expr = b_.EvaluatePartial(env).ToExpression();

  // Negative side, no margin.
  std::vector<symbolic::RationalFunction> rationals;
  std::optional<VectorX<symbolic::Polynomial>> unit_length_vector;
  capsule.OnPlaneSide(a_, b_, X_AB_multilinear, rational_forward_kin_,
                      std::nullopt, PlaneSide::kNegative, &rationals,
                      &unit_length_vector);
  EXPECT_TRUE(unit_length_vector.has_value());
  EXPECT_EQ(unit_length_vector->rows(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ((*unit_length_vector)(i), a_(i));
  }
  EXPECT_EQ(rationals.size(), 2);
  const Capsule& capsule_shape =
      static_cast<const Capsule&>(model_inspector.GetShape(body2_capsule_));
  Eigen::Matrix<double, 3, 2> p_GS;
  p_GS.col(0) << 0, 0, capsule_shape.length() / 2;
  p_GS.col(1) << 0, 0, -capsule_shape.length() / 2;
  const Eigen::Matrix<double, 3, 2> p_BS = capsule.X_BG() * p_GS;
  Eigen::Matrix<double, 3, 2> p_AS;
  plant_->CalcPointsPositions(
      *plant_context_, plant_->get_body(geometry_body).body_frame(), p_BS,
      plant_->get_body(expressed_body).body_frame(), &p_AS);
  for (int i = 0; i < 2; ++i) {
    symbolic::Expression expr_expected =
        -capsule_shape.radius() - a_expr.dot(p_AS.col(i)) - b_expr;
    CheckRationalExpression(rationals[i], env, expr_expected);
  }

  // Positive side, with margin.
  rationals.clear();
  const symbolic::Variable separating_margin("delta");
  capsule.OnPlaneSide(a_, b_, X_AB_multilinear, rational_forward_kin_,
                      separating_margin, PlaneSide::kPositive, &rationals,
                      &unit_length_vector);
  EXPECT_TRUE(unit_length_vector.has_value());
  EXPECT_EQ(unit_length_vector->rows(), 3);
  for (int i = 0; i < 2; ++i) {
    const symbolic::Expression expr_expected = a_expr.dot(p_AS.col(i)) +
                                               b_expr - capsule_shape.radius() -
                                               separating_margin;
    CheckRationalExpression(rationals[i], env, expr_expected);
  }
}

GTEST_TEST(DistanceToHalfspace, Test) {
  // Construct a plant with many collision geometries (including halfspace).
  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>* plant;
  geometry::SceneGraph<double>* scene_graph;
  std::tie(plant, scene_graph) =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.);

  ProximityProperties proximity_properties{};
  // C-IRIS doesn't care about robot dynamics. Use arbitrary material
  // properties.
  AddContactMaterial(0.1, 250.0, multibody::CoulombFriction<double>{0.9, 0.5},
                     &proximity_properties);
  // C-IRIS only considers robot kinematics, not dynamics. So we use an
  // arbitrary inertia.
  const multibody::SpatialInertia<double> spatial_inertia(
      1, Eigen::Vector3d::Zero(),
      multibody::UnitInertia<double>(0.01, 0.01, 0.01, 0, 0, 0));

  std::vector<multibody::BodyIndex> body_indices;
  for (int i = 0; i < 5; ++i) {
    body_indices.push_back(
        plant->AddRigidBody("body" + std::to_string(i), spatial_inertia)
            .index());
  }
  const Eigen::Vector3d box_size(0.05, 0.02, 0.03);
  const geometry::GeometryId body0_box = plant->RegisterCollisionGeometry(
      plant->get_body(body_indices[0]),
      math::RigidTransform(Eigen::Vector3d(0.01, 0.02, 0.03)),
      Box(box_size(0), box_size(1), box_size(2)), "body0_box",
      proximity_properties);

  const geometry::GeometryId body1_sphere = plant->RegisterCollisionGeometry(
      plant->get_body(body_indices[1]),
      math::RigidTransform(Eigen::Vector3d(0.02, -0.01, 0.05)), Sphere(0.01),
      "body1_sphere", proximity_properties);

  const double capsule_radius = 0.04;
  const double capsule_length = 0.1;
  const geometry::GeometryId body2_capsule = plant->RegisterCollisionGeometry(
      plant->get_body(body_indices[2]),
      math::RigidTransformd(math::RollPitchYawd(0.05, -0.02, 0.01),
                            Eigen::Vector3d(0.01, 0.02, -0.03)),
      Capsule(capsule_radius, capsule_length), "body2_capsule",
      proximity_properties);

  const geometry::GeometryId body3_halfspace = plant->RegisterCollisionGeometry(
      plant->get_body(body_indices[3]),
      math::RigidTransformd(math::RollPitchYawd(0.02, -0.1, 0.05),
                            Eigen::Vector3d(0.03, -0.01, 0.02)),
      HalfSpace(), "body3_halfspace", proximity_properties);
  plant->Finalize();
  auto diagram = builder.Build();

  const auto& model_inspector = scene_graph->model_inspector();

  auto diagram_context = diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      plant->GetMyMutableContextFromRoot(diagram_context.get());
  Eigen::VectorXd q = plant->GetPositions(plant_context);
  q.head<7>() << 0, 1, 0, 0, 0.2, -0.1, 0.3;
  q.segment<7>(7) << 0, 0, 1, 0, -0.5, 0.3, 0.2;
  q.segment<7>(14) << 0.5, 0.5, -0.5, 0.5, 0.3, -0.5, 0.2;
  q.segment<7>(21) << 0.5, -0.5, -0.5, 0.5, -0.1, 0.4, 1;
  q.tail<7>() << 0.5, -0.5, -0.5, -0.5, 0.3, -0.2, 0.4;
  plant->SetPositions(&plant_context, q);

  // Test the distance for each geometry.
  const CollisionGeometry sphere(&(model_inspector.GetShape(body1_sphere)),
                                 body_indices[1], body1_sphere,
                                 model_inspector.GetPoseInFrame(body1_sphere));

  // Expressed body is where the halfspace is welded to.
  const multibody::BodyIndex expressed_body = body_indices[3];
  const auto X_HalfspaceE =
      model_inspector.GetPoseInFrame(body3_halfspace).inverse();

  Eigen::Vector3d a = X_HalfspaceE.rotation().matrix().row(2).transpose();
  double b = X_HalfspaceE.translation()(2);
  // Now multiply a and b arbitrarily to make a.norm() != 1.
  a *= 1.5;
  b *= 1.5;

  const auto& query_port = plant->get_geometry_query_input_port();
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<double>>(plant_context);
  const double kTol = 1E-10;

  {
    // Distance to sphere.
    const double distance =
        DistanceToHalfspace(sphere, a, b, expressed_body, PlaneSide::kPositive,
                            *plant, plant_context);
    const double distance_expected =
        query_object
            .ComputeSignedDistancePairClosestPoints(body3_halfspace,
                                                    body1_sphere)
            .distance;
    EXPECT_NEAR(distance, distance_expected, kTol);
  }

  {
    // Distance to box
    const CollisionGeometry box(&(model_inspector.GetShape(body0_box)),
                                body_indices[0], body0_box,
                                model_inspector.GetPoseInFrame(body0_box));
    const double distance = DistanceToHalfspace(
        box, a, b, expressed_body, PlaneSide::kNegative, *plant, plant_context);
    // SceneGraph doesn't support distance between box and halfspace yet.
    Eigen::Matrix<double, 3, 8> p_GV;
    // clang-format off
    p_GV << -1, -1, -1, -1, 1, 1, 1, 1,
            1, 1, -1, -1, 1, 1, -1, -1,
            1, -1, 1, -1, 1, -1, 1, -1;
    // clang-format on
    p_GV.row(0) *= box_size(0) / 2;
    p_GV.row(1) *= box_size(1) / 2;
    p_GV.row(2) *= box_size(2) / 2;
    // Now compute the position of these vertices V in the halfspace frame H.
    Eigen::Matrix<double, 3, 8> p_EV;
    plant->CalcPointsPositions(
        plant_context, plant->get_body(body_indices[0]).body_frame(),
        model_inspector.GetPoseInFrame(body0_box) * p_GV,
        plant->get_body(expressed_body).body_frame(), &p_EV);
    const Eigen::Matrix<double, 3, 8> p_HV = X_HalfspaceE * p_EV;
    const double distance_expected = (-p_HV.row(2)).minCoeff();
    EXPECT_NEAR(distance, distance_expected, kTol);
  }

  {
    // Distance to capsule
    const CollisionGeometry capsule(
        &(model_inspector.GetShape(body2_capsule)), body_indices[2],
        body2_capsule, model_inspector.GetPoseInFrame(body2_capsule));
    const double distance =
        DistanceToHalfspace(capsule, a, b, expressed_body, PlaneSide::kNegative,
                            *plant, plant_context);
    // SceneGraph doesn't support distance between capsule and halfspace yet.
    // Compute the position of capsule sphere center S in the halfspace frame.
    Eigen::Matrix<double, 3, 2> p_GS;
    p_GS.col(0) << 0, 0, capsule_length / 2;
    p_GS.col(1) << 0, 0, -capsule_length / 2;
    Eigen::Matrix<double, 3, 2> p_ES;
    plant->CalcPointsPositions(
        plant_context, plant->get_body(body_indices[2]).body_frame(),
        model_inspector.GetPoseInFrame(body2_capsule) * p_GS,
        plant->get_body(expressed_body).body_frame(), &p_ES);
    const Eigen::Matrix<double, 3, 2> p_HS = X_HalfspaceE * p_ES;

    const double distance_expected = (-p_HS.row(2)).minCoeff() - capsule_radius;
    EXPECT_NEAR(distance, distance_expected, kTol);
  }
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
