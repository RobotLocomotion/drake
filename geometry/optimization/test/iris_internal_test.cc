#include "drake/geometry/optimization/iris_internal.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {

class TwoSpheres : public testing::Test {
 public:
  TwoSpheres() {
    systems::DiagramBuilder<double> builder{};
    std::tie(plant_, scene_graph_) =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
    // Add two spheres with arbitrary mass/inertia.
    const double mass{1};
    const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
    const multibody::RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
    const multibody::SpatialInertia<double> M_AAo_A =
        multibody::SpatialInertia<double>::MakeFromCentralInertia(
            mass, p_AoAcm_A, I_AAcm_A);
    sphere1_index_ = plant_->AddRigidBody("sphere1", M_AAo_A).index();
    sphere2_index_ = plant_->AddRigidBody("sphere2", M_AAo_A).index();
    sphere1_radius_ = 0.1;
    sphere2_radius_ = 0.2;
    plant_->RegisterCollisionGeometry(
        plant_->get_body(sphere1_index_), {}, geometry::Sphere(sphere1_radius_),
        "sphere1_collision", multibody::CoulombFriction<double>());
    plant_->RegisterCollisionGeometry(
        plant_->get_body(sphere2_index_), {}, geometry::Sphere(sphere2_radius_),
        "sphere2_collision", multibody::CoulombFriction<double>());
    plant_->Finalize();
    diagram_ = builder.Build();
  }

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  multibody::BodyIndex sphere1_index_;
  multibody::BodyIndex sphere2_index_;
  double sphere1_radius_;
  double sphere2_radius_;
};

TEST_F(TwoSpheres, TestSamePointConstraint) {
  auto diagram_context = diagram_->CreateDefaultContext();
  auto& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context.get());
  SamePointConstraint dut(plant_, plant_context);
  dut.set_frameA(&(plant_->get_body(sphere1_index_).body_frame()));
  dut.set_frameB(&(plant_->get_body(sphere2_index_).body_frame()));

  Eigen::VectorXd x(dut.num_vars());
  x.head(plant_->num_positions()) = plant_->GetPositions(plant_context);
  Eigen::Vector3d p_AA(0.1, 0.2, 0.3);
  Eigen::Vector3d p_BB(0.2, 0.3, 0.4);
  x.segment<3>(4) << 0.1, 0.1, 0.1;
  x.segment<3>(11) << 0, 0, 0;
  x.tail<6>() << p_AA, p_BB;
  Eigen::VectorXd y(dut.num_outputs());
  dut.Eval(x, &y);
  EXPECT_TRUE(CompareMatrices(y, Eigen::Vector3d::Zero(), 1E-10));
}

// A planar robot with three links (including one base link), connected by two
// revolute joints.
class TwoLinks : public ::testing::Test {
 public:
  TwoLinks() {
    systems::DiagramBuilder<double> builder{};
    std::tie(plant_, scene_graph_) =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
    // Add two spheres with arbitrary mass/inertia.
    const double mass{1};
    const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
    const multibody::RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
    const multibody::SpatialInertia<double> M_AAo_A =
        multibody::SpatialInertia<double>::MakeFromCentralInertia(
            mass, p_AoAcm_A, I_AAcm_A);
    for (int i = 0; i < 3; ++i) {
      body_indices_.push_back(
          plant_->AddRigidBody("link" + std::to_string(i), M_AAo_A).index());
    }
    plant_->WeldFrames(plant_->world_frame(),
                       plant_->get_body(body_indices_[0]).body_frame());
    for (int i = 1; i < 3; ++i) {
      plant_->AddJoint<multibody::RevoluteJoint>(
          "joint" + std::to_string(i - 1),
          plant_->get_body(body_indices_[i - 1]),
          math::RigidTransformd(0.1 * Eigen::Vector3d::UnitY()),
          plant_->get_body(body_indices_[i]), {}, Eigen::Vector3d::UnitZ());
    }
    plant_->Finalize();
    diagram_ = builder.Build();
  }

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  std::vector<multibody::BodyIndex> body_indices_;
};

TEST_F(TwoLinks, TestClosestCollisionProgram) {
  auto diagram_context = diagram_->CreateDefaultContext();
  auto& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context.get());
  auto same_point_constraint =
      std::make_shared<SamePointConstraint>(plant_, plant_context);

  const double radius_A = 0.1;
  const double radius_B = 0.1;
  Hyperellipsoid setA(1. / radius_A * Eigen::Matrix3d::Identity(),
                      Eigen::Vector3d(0, 0.5, 0));
  Hyperellipsoid setB(1. / radius_B * Eigen::Matrix3d::Identity(),
                      Eigen::Vector3d(0, -0.5, 0));
  Hyperellipsoid E(0.2 * Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero());
  Eigen::MatrixXd A(0, 2);
  Eigen::VectorXd b(0);
  ClosestCollisionProgram dut(same_point_constraint,
                              plant_->get_body(body_indices_[2]).body_frame(),
                              plant_->world_frame(), setA, setB, E, A, b);

  Eigen::Vector2d q_guess(0, 0.5);

  Eigen::VectorXd closest;
  bool found_collision =
      dut.Solve(solvers::IpoptSolver(), q_guess, std::nullopt, &closest);
  EXPECT_TRUE(found_collision);
  // Check the solution.
  Eigen::Vector3d p_WA;
  plant_->SetPositions(&plant_context, closest);
  plant_->CalcPointsPositions(plant_context,
                              plant_->get_body(body_indices_[2]).body_frame(),
                              setA.center(), plant_->world_frame(), &p_WA);
  Eigen::Vector3d p_WB = setB.center();
  // In collision.
  EXPECT_LE((p_WA - p_WB).norm(), radius_A + radius_B + 1E-5);

  // Now test passing a solver_options.
  solvers::SolverOptions solver_options;
  solver_options.SetOption(solvers::IpoptSolver::id(), "max_iter", 1);
  found_collision =
      dut.Solve(solvers::IpoptSolver(), q_guess, solver_options, &closest);
  EXPECT_FALSE(found_collision);
}
}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
