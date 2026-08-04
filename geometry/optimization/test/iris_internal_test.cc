#include "drake/geometry/optimization/iris_internal.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {

class ThreeSpheres : public testing::Test {
 public:
  ThreeSpheres() {
    systems::DiagramBuilder<double> builder{};
    std::tie(plant_, scene_graph_) =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
    // Add two spheres with arbitrary mass/inertia.
    std::string xml = R"""(
    <mujoco model="twospheres">
      <worldbody>
         <body name="sphere1"> <geom type="sphere" size="0.1"/><freejoint name="freejoint1"/></body>
         <body name="sphere2"> <geom type="sphere" size="0.2"/><freejoint name="freejoint2"/></body>
      </worldbody>
    </mujoco>
    )""";
    multibody::Parser parser(plant_);
    parser.AddModelsFromString(xml, "xml");
    sphere1_index_ = plant_->GetBodyByName("sphere1").index();
    sphere2_index_ = plant_->GetBodyByName("sphere2").index();
    plant_->Finalize();
    diagram_ = builder.Build();
  }

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  multibody::BodyIndex sphere1_index_;
  multibody::BodyIndex sphere2_index_;
};

TEST_F(ThreeSpheres, TestSamePointConstraint) {
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
class ThreeLinks : public ::testing::Test {
 public:
  ThreeLinks() {
    systems::DiagramBuilder<double> builder{};
    std::tie(plant_, scene_graph_) =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
    std::string xml = R"""(
    <mujoco model="threelinks">
    <worldbody>
      <body name="link0"></body>
      <body name="link1" pos="0 0.1 0"><joint name="joint0" type="hinge" axis="0 0 1"/>
        <body name="link2" pos="0 0.1 0"><joint name="joint1" type="hinge" axis="0 0 1"/></body>
      </body>
    </worldbody>
    </mujoco>
    )""";
    multibody::Parser parser(plant_);
    parser.AddModelsFromString(xml, "xml");
    for (int i = 0; i < 3; ++i) {
      body_indices_[i] =
          plant_->GetBodyByName("link" + std::to_string(i)).index();
    }
    plant_->Finalize();
    diagram_ = builder.Build();
  }

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  std::array<multibody::BodyIndex, 3> body_indices_;
};

TEST_F(ThreeLinks, TestClosestCollisionProgram) {
  auto diagram_context = diagram_->CreateDefaultContext();
  auto& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context.get());
  auto same_point_constraint =
      std::make_shared<SamePointConstraint>(plant_, plant_context);

  const double radius_A = 0.1;
  const double radius_B = 0.1;
  Hyperellipsoid setA(1.0 / radius_A * Eigen::Matrix3d::Identity(),
                      Eigen::Vector3d(0, 0.5, 0));
  Hyperellipsoid setB(1.0 / radius_B * Eigen::Matrix3d::Identity(),
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
