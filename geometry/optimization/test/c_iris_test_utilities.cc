#include "drake/geometry/optimization/test/c_iris_test_utilities.h"

#include <string>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/sos_basis_generator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {
namespace optimization {

CIrisToyRobotTest::CIrisToyRobotTest() {
  systems::DiagramBuilder<double> builder;
  std::tie(plant_, scene_graph_) =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0);

  ProximityProperties proximity_properties{};
  // C-IRIS doesn't care about robot dynamics. Use arbitrary material
  // properties.
  AddContactMaterial(0.1, 250.0, multibody::CoulombFriction<double>{0.9, 0.5},
                     &proximity_properties);

  world_box_ = plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(math::RollPitchYawd(0.5, 0.2, -0.3),
                           Eigen::Vector3d(0.2, -0.5, 0.1)),
      geometry::Box(0.02, 0.03, 0.01), "world_box", proximity_properties);
  world_cylinder_ = plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(Eigen::Vector3d(-0.1, -0.1, 0.2)),
      Cylinder(0.02, 0.1), "world_cylinder", proximity_properties);

  // body0
  body_indices_.push_back(plant_->AddRigidBody("body0").index());
  const multibody::RigidBody<double>& body0 =
      plant_->get_body(body_indices_[0]);
  plant_->AddJoint<multibody::WeldJoint>(
      "joint0", plant_->world_body(),
      math::RigidTransformd(Eigen::Vector3d(0.1, 0.2, 0)), body0,
      math::RigidTransformd(math::RollPitchYawd(0.1, 0.5, 0.2),
                            Eigen::Vector3d::Zero()),
      math::RigidTransformd(Eigen::Vector3d(0.05, 0.1, 0.05)));
  body0_box_ = plant_->RegisterCollisionGeometry(
      body0, math::RigidTransform(Eigen::Vector3d(0.1, 0.05, -0.05)),
      Box(0.05, 0.1, 0.04), "body0_box", proximity_properties);
  body0_sphere_ = plant_->RegisterCollisionGeometry(
      body0, math::RigidTransform(Eigen::Vector3d(0.01, -0.02, 0)),
      Sphere(0.08), "body0_sphere", proximity_properties);

  // body1
  body_indices_.push_back(plant_->AddRigidBody("body1").index());
  const multibody::RigidBody<double>& body1 =
      plant_->get_body(body_indices_[1]);
  const auto& joint1 = plant_->AddJoint<multibody::RevoluteJoint>(
      "joint1", body0,
      math::RigidTransformd(math::RollPitchYawd(0.1, 0.2, -0.1),
                            Eigen::Vector3d(0.2, 0.4, 0.1)),
      body1, math::RigidTransformd(Eigen::Vector3d(0.1, 0.2, -0.05)),
      Eigen::Vector3d::UnitY());
  plant_->get_mutable_joint(joint1.index())
      .set_position_limits(Vector1d(-0.8 * M_PI), Vector1d(0.7 * M_PI));
  body1_capsule_ = plant_->RegisterCollisionGeometry(
      body1, math::RigidTransformd(Eigen::Vector3d(0.02, -0.1, 0.05)),
      Capsule(0.08, 0.2), "body1_capsule", proximity_properties);
  const std::string convex_obj =
      FindResourceOrThrow("drake/geometry/optimization/test/convex.obj");
  body1_convex_ = plant_->RegisterCollisionGeometry(
      body1,
      math::RigidTransformd(math::RollPitchYawd(0.05, -0.03, 0),
                            Eigen::Vector3d(0.04, 0.02, 0.05)),
      Convex(convex_obj), "body1_convex", proximity_properties);

  // body2
  body_indices_.push_back(plant_->AddRigidBody("body2").index());
  const auto& body2 = plant_->get_body(body_indices_[2]);
  const auto& joint2 = plant_->AddJoint<multibody::PrismaticJoint>(
      "joint2", body1, math::RigidTransformd(Eigen::Vector3d(0.2, 0, 0)), body2,
      math::RigidTransformd(math::RollPitchYawd(0.1, -0.2, 0.1),
                            Eigen::Vector3d(0.02, 0.1, 0.03)),
      Eigen::Vector3d::UnitX());
  plant_->get_mutable_joint(joint2.index())
      .set_position_limits(Vector1d(-2.4), Vector1d(2.9));
  body2_capsule_ = plant_->RegisterCollisionGeometry(
      body2, math::RigidTransform(Eigen::Vector3d(0.02, 0.05, 0)),
      Capsule(0.06, 0.1), "body2_capsule", proximity_properties);
  body2_sphere_ = plant_->RegisterCollisionGeometry(
      body2, math::RigidTransform(Eigen::Vector3d(0.01, 0.04, 0.02)),
      Sphere(0.07), "body2_sphere", proximity_properties);

  // body3
  body_indices_.push_back(plant_->AddRigidBody("body3").index());
  const auto& body3 = plant_->get_body(body_indices_[3]);
  const auto& joint3 = plant_->AddJoint<multibody::RevoluteJoint>(
      "joint3", body0, math::RigidTransformd(Eigen::Vector3d(0, 0.05, 0.1)),
      body3,
      math::RigidTransformd(math::RollPitchYawd(0.1, -0.1, 0.2),
                            Eigen::Vector3d(0.1, 0.2, -0.05)),
      Eigen::Vector3d::UnitY());
  plant_->get_mutable_joint(joint3.index())
      .set_position_limits(Vector1d(-0.7 * M_PI), Vector1d(0.6 * M_PI));
  body3_box_ = plant_->RegisterCollisionGeometry(
      body3, math::RigidTransformd(Eigen::Vector3d(-0.1, -0.1, 0.02)),
      Box(0.02, 0.05, 0.02), "body3_box", proximity_properties);
  body3_cylinder_ = plant_->RegisterCollisionGeometry(
      body3, math::RigidTransformd(Eigen::Vector3d(0.1, 0.02, 0.2)),
      Cylinder(0.04, 0.05), "body3_cylinder", proximity_properties);

  plant_->Finalize();
  diagram_ = builder.Build();
}

CIrisRobotPolytopicGeometryTest::CIrisRobotPolytopicGeometryTest() {
  systems::DiagramBuilder<double> builder;
  std::tie(plant_, scene_graph_) =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

  ProximityProperties proximity_properties{};
  // C-IRIS doesn't care about robot dynamics. Use arbitrary material
  // properties.
  AddContactMaterial(0.1, 250.0, multibody::CoulombFriction<double>{0.9, 0.5},
                     &proximity_properties);

  world_boxes_.push_back(plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(math::RollPitchYawd(0.5, 0.2, -0.3),
                           Eigen::Vector3d(0.2, -0.5, 0.1)),
      geometry::Box(0.02, 0.03, 0.01), "world_box0", proximity_properties));
  world_boxes_.push_back(plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(math::RollPitchYawd(0.1, 0.2, -0.),
                           Eigen::Vector3d(0.2, 0.3, 0.1)),
      geometry::Box(0.02, 0.1, 0.05), "world_box1", proximity_properties));
  world_boxes_.push_back(plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(math::RollPitchYawd(0.1, 0.2, -0.),
                           Eigen::Vector3d(0.2, 0.2, 0.1)),
      geometry::Box(0.04, 0.1, 0.05), "world_box2", proximity_properties));
  const std::string convex_obj =
      FindResourceOrThrow("drake/geometry/optimization/test/convex.obj");
  world_convex_ = plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(Eigen::Vector3d(-0.1, -0.5, 0.2)),
      Convex(convex_obj), "world_convex", proximity_properties);

  auto add_body = [this, &proximity_properties](
                      const math::RigidTransformd& X_PF,
                      const math::RigidTransformd& X_BM,
                      const Eigen::Vector3d& axis, double theta_lower,
                      double theta_upper, const math::RigidTransformd& X_BG,
                      const Eigen::Vector3d& box_size) {
    const int body_index = this->body_indices_.size();
    const auto& parent_body =
        this->body_indices_.empty()
            ? this->plant_->world_body()
            : this->plant_->get_body(this->body_indices_.back());
    this->body_indices_.push_back(
        this->plant_->AddRigidBody("body" + std::to_string(body_index))
            .index());

    const auto& body = this->plant_->get_body(this->body_indices_.back());
    const auto& joint = this->plant_->AddJoint<multibody::RevoluteJoint>(
        "joint" + std::to_string(body_index), parent_body, X_PF, body, X_BM,
        axis);
    plant_->get_mutable_joint(joint.index())
        .set_position_limits(Vector1d(theta_lower), Vector1d(theta_upper));
    this->body_boxes_.push_back(this->plant_->RegisterCollisionGeometry(
        body, X_BG, Box(box_size(0), box_size(1), box_size(2)),
        "body" + std::to_string(body_index) + "_box", proximity_properties));
  };

  // body0
  add_body(math::RigidTransformd(Eigen::Vector3d(0.1, 0.2, 0)),
           math::RigidTransformd(Eigen::Vector3d(0.05, 0.01, 0)),
           Eigen::Vector3d::UnitX(), -0.5 * M_PI, 0.8 * M_PI,
           math::RigidTransformd(Eigen::Vector3d(0.05, 0.1, 0)),
           Eigen::Vector3d(0.1, 0.15, 0.1));
  // body1
  add_body(math::RigidTransformd(Eigen::Vector3d(0.05, 0, 0.1)),
           math::RigidTransformd(), Eigen::Vector3d::UnitY(), -0.6 * M_PI,
           0.7 * M_PI, math::RigidTransformd(Eigen::Vector3d(0.02, 0, 0.05)),
           Eigen::Vector3d(0.04, 0.05, 0.08));
  // body2
  add_body(math::RigidTransformd(Eigen::Vector3d(0., 0.2, 0.1)),
           math::RigidTransformd(), Eigen::Vector3d::UnitX(), -0.3 * M_PI,
           0.7 * M_PI, math::RigidTransformd(Eigen::Vector3d(0.0, 0.1, 0.05)),
           Eigen::Vector3d(0.04, 0.1, 0.08));
  // body3
  add_body(math::RigidTransformd(Eigen::Vector3d(0.1, 0.2, -0.1)),
           math::RigidTransformd(), Eigen::Vector3d::UnitZ(), -0.5 * M_PI,
           0.7 * M_PI, math::RigidTransformd(Eigen::Vector3d(0.0, 0.1, 0.04)),
           Eigen::Vector3d(0.05, 0.1, 0.02));

  plant_->Finalize();
  diagram_ = builder.Build();
}

void CspaceFreePolytopeTester::FindRedundantInequalities(
    const Eigen::MatrixXd& C, const Eigen::VectorXd& d,
    const Eigen::VectorXd& s_lower, const Eigen::VectorXd& s_upper,
    double tighten, std::unordered_set<int>* C_redundant_indices,
    std::unordered_set<int>* s_lower_redundant_indices,
    std::unordered_set<int>* s_upper_redundant_indices) const {
  cspace_free_polytope_->FindRedundantInequalities(
      C, d, s_lower, s_upper, tighten, C_redundant_indices,
      s_lower_redundant_indices, s_upper_redundant_indices);
}

CspaceFreePolytope::SeparationCertificateProgram
CspaceFreePolytopeTester::ConstructPlaneSearchProgram(
    const PlaneSeparatesGeometries& plane_geometries,
    const VectorX<symbolic::Polynomial>& d_minus_Cs,
    const std::unordered_set<int>& C_redundant_indices,
    const std::unordered_set<int>& s_lower_redundant_indices,
    const std::unordered_set<int>& s_upper_redundant_indices) const {
  return cspace_free_polytope_->ConstructPlaneSearchProgram(
      plane_geometries, d_minus_Cs, C_redundant_indices,
      s_lower_redundant_indices, s_upper_redundant_indices);
}

std::vector<std::optional<CspaceFreePolytope::SeparationCertificateResult>>
CspaceFreePolytopeTester::FindSeparationCertificateGivenPolytope(
    const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::VectorXd>& d,
    const CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions&
        options) const {
  return cspace_free_polytope_->FindSeparationCertificateGivenPolytope(
      ignored_collision_pairs, C, d, options);
}

int CspaceFreePolytopeTester::GetGramVarSizeForPolytopeSearchProgram(
    const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
    bool search_s_bounds_lagrangians) const {
  return cspace_free_polytope_->GetGramVarSizeForPolytopeSearchProgram(
      ignored_collision_pairs, search_s_bounds_lagrangians);
}

std::unique_ptr<solvers::MathematicalProgram>
CspaceFreePolytopeTester::InitializePolytopeSearchProgram(
    const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
    const MatrixX<symbolic::Variable>& C, const VectorX<symbolic::Variable>& d,
    const VectorX<symbolic::Polynomial> d_minus_Cs,
    const std::vector<
        std::optional<CspaceFreePolytope::SeparationCertificateResult>>&
        certificates_vec,
    bool search_s_bounds_lagrangians, int gram_total_size,
    std::unordered_map<int, CspaceFreePolytope::SeparationCertificate>*
        new_certificates) const {
  return cspace_free_polytope_->InitializePolytopeSearchProgram(
      ignored_collision_pairs, C, d, d_minus_Cs, certificates_vec,
      search_s_bounds_lagrangians, gram_total_size, new_certificates);
}

void CspaceFreePolytopeTester::AddEllipsoidContainmentConstraint(
    solvers::MathematicalProgram* prog, const Eigen::MatrixXd& Q,
    const Eigen::VectorXd& s0, const MatrixX<symbolic::Variable>& C,
    const VectorX<symbolic::Variable>& d,
    const VectorX<symbolic::Variable>& ellipsoid_margins) const {
  cspace_free_polytope_->AddEllipsoidContainmentConstraint(prog, Q, s0, C, d,
                                                           ellipsoid_margins);
}

void CspaceFreePolytopeTester::AddCspacePolytopeContainment(
    solvers::MathematicalProgram* prog, const MatrixX<symbolic::Variable>& C,
    const VectorX<symbolic::Variable>& d,
    const Eigen::MatrixXd& s_inner_pts) const {
  cspace_free_polytope_->AddCspacePolytopeContainment(prog, C, d, s_inner_pts);
}

std::optional<CspaceFreePolytope::FindPolytopeGivenLagrangianResult>
CspaceFreePolytopeTester::FindPolytopeGivenLagrangian(
    const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
    const MatrixX<symbolic::Variable>& C, const VectorX<symbolic::Variable>& d,
    const VectorX<symbolic::Polynomial>& d_minus_Cs,
    const std::vector<
        std::optional<CspaceFreePolytope::SeparationCertificateResult>>&
        certificates_vec,
    const Eigen::MatrixXd& Q, const Eigen::VectorXd& s0,
    const VectorX<symbolic::Variable>& ellipsoid_margins, int gram_total_size,
    const CspaceFreePolytope::FindPolytopeGivenLagrangianOptions& options,
    std::unordered_map<int, CspaceFreePolytope::SeparationCertificateResult>*
        certificates_result) const {
  return cspace_free_polytope_->FindPolytopeGivenLagrangian(
      ignored_collision_pairs, C, d, d_minus_Cs, certificates_vec, Q, s0,
      ellipsoid_margins, gram_total_size, options, certificates_result);
}

HPolyhedron CspaceFreePolytopeTester::GetPolyhedronWithJointLimits(
    const Eigen::MatrixXd& C, const Eigen::VectorXd& d) const {
  return cspace_free_polytope_->GetPolyhedronWithJointLimits(C, d);
}

bool InCollision(const multibody::MultibodyPlant<double>& plant,
                 const systems::Context<double>& plant_context) {
  const auto& query_port = plant.get_geometry_query_input_port();
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<double>>(plant_context);
  const std::vector<geometry::SignedDistancePair<double>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints(
              0.5 /* max_distance */);
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    if (signed_distance_pair.distance <= 0) {
      return true;
    }
  }
  return false;
}

// Evaluate the polynomial at a batch of samples, check if all evaluated results
// are positive.
// @param x_samples Each column is a sample of indeterminates.
void CheckPositivePolynomialBySamples(
    const symbolic::Polynomial& poly,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& indeterminates,
    const Eigen::Ref<const Eigen::MatrixXd>& x_samples) {
  EXPECT_TRUE(
      (poly.EvaluateIndeterminates(indeterminates, x_samples).array() >= 0)
          .all());
}

bool IsPolynomialSos(const symbolic::Polynomial& p, double tol) {
  DRAKE_DEMAND(p.decision_variables().empty());
  if (p.monomial_to_coefficient_map().empty()) {
    // p = 0.
    return true;
  } else if (p.monomial_to_coefficient_map().size() == 1 &&
             p.monomial_to_coefficient_map().contains(symbolic::Monomial())) {
    // p is a constant
    symbolic::Environment env;
    const double constant =
        p.monomial_to_coefficient_map().at(symbolic::Monomial()).Evaluate(env);
    return constant >= -tol;
  }
  solvers::MathematicalProgram prog;
  VectorX<symbolic::Variable> indeterminates_vec(p.indeterminates().size());
  prog.AddIndeterminates(p.indeterminates());
  if (tol == 0) {
    prog.AddSosConstraint(p);
  } else {
    const VectorX<symbolic::Monomial> monomial_basis =
        solvers::ConstructMonomialBasis(p);
    const auto pair = prog.NewSosPolynomial(
        monomial_basis,
        solvers::MathematicalProgram::NonnegativePolynomial::kSos);
    const symbolic::Polynomial poly_diff = pair.first - p;
    for (const auto& term : poly_diff.monomial_to_coefficient_map()) {
      prog.AddLinearConstraint(term.second, -tol, tol);
    }
  }
  const auto result = solvers::Solve(prog);
  return result.is_success();
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
