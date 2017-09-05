#include <memory>

#include <gtest/gtest.h>

#include "drake/examples/acrobot/multibody/acrobot_multibody_plant.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using multibody::Link;
using multibody::RevoluteJoint;
using std::unique_ptr;
using systems::Context;

// Tests that the hand-derived dynamics (from the textbook) match the dynamics
// generated from the urdf via the RigidBodyPlant class.
GTEST_TEST(AcrobotMultibodyPlant, AllTests) {
  AcrobotMultibodyPlant<double> acrobot_plant;
  multibody::benchmarks::Acrobot<double> acrobot_benchmark(
      Vector3d::UnitZ(), Vector3d::UnitY(),
      acrobot_plant.m1(), acrobot_plant.m2(),
      acrobot_plant.l1(), acrobot_plant.l2(),
      acrobot_plant.lc1(), acrobot_plant.lc2(),
      acrobot_plant.Ic1(), acrobot_plant.Ic2(),
      acrobot_plant.b1(), acrobot_plant.b2(),
      acrobot_plant.g());

  const Link<double>& link1 = acrobot_plant.get_link1();
  const Link<double>& link2 = acrobot_plant.get_link2();
  const RevoluteJoint<double>& shoulder = acrobot_plant.get_shoulder_joint();
  const RevoluteJoint<double>& elbow = acrobot_plant.get_elbow_joint();

  unique_ptr<Context<double>> context = acrobot_plant.CreateDefaultContext();

  const multibody::MultibodyModeler<double>& modeler =
      acrobot_plant.get_modeler();
  const multibody::MultibodyTree<double>& model =
      modeler.get_multibody_tree_model();
  multibody::PositionKinematicsCache<double> pc(model.get_topology());
  multibody::BodyNodeIndex link1_node_index =
      modeler.get_link_body(link1).get_node_index();
  multibody::BodyNodeIndex link2_node_index =
      modeler.get_link_body(link2).get_node_index();

  double theta1 = shoulder.get_angle(*context);
  double theta2 = elbow.get_angle(*context);

  double theta1dot = 1.0; shoulder.set_angular_rate(context.get(), theta1dot);
  double theta2dot = -0.5; elbow.set_angular_rate(context.get(), theta2dot);

  PRINT_VAR(theta1);
  PRINT_VAR(theta2);

  Matrix2d H = acrobot_plant.MatrixH(*context);
  Matrix2d H_expected = acrobot_benchmark.CalcMassMatrix(theta2);

  PRINT_VARn(H);
  PRINT_VARn(H_expected);

  PRINT_VAR(theta1dot);
  PRINT_VAR(shoulder.get_angular_rate(*context));
  PRINT_VAR(theta2dot);
  PRINT_VAR(elbow.get_angular_rate(*context));

  Vector2d C = acrobot_plant.VectorC(*context);
  Vector2d C_expected =
      acrobot_benchmark.CalcCoriolisVector(theta1, theta2,
                                           theta1dot, theta2dot);
  PRINT_VAR(C.transpose());
  PRINT_VAR(C_expected.transpose());

  PRINT_VAR(link1_node_index);
  PRINT_VAR(link2_node_index);
  model.CalcPositionKinematicsCache(*context, &pc);
  PRINT_VARn(pc.get_mutable_X_WB(link1_node_index).matrix());
  PRINT_VARn(acrobot_benchmark.CalcLink1PoseInWorldFrame(theta1).matrix());
  PRINT_VARn(pc.get_mutable_X_WB(link2_node_index).matrix());
  PRINT_VARn(acrobot_benchmark.CalcLink2PoseInWorldFrame(theta1, theta2).matrix());

  theta1 = M_PI / 3.0;
  theta2 = M_PI / 4.0;
  PRINT_VAR(theta1);
  PRINT_VAR(theta2);
  shoulder.set_angle(context.get(), theta1);
  elbow.set_angle(context.get(), theta2);
  PRINT_VAR(shoulder.get_angle(*context));
  PRINT_VAR(elbow.get_angle(*context));

  H = acrobot_plant.MatrixH(*context);
  H_expected = acrobot_benchmark.CalcMassMatrix(theta2);

  PRINT_VARn(H);
  PRINT_VARn(H_expected);

  C = acrobot_plant.VectorC(*context);
  C_expected =
      acrobot_benchmark.CalcCoriolisVector(theta1, theta2,
                                           theta1dot, theta2dot);
  PRINT_VAR(C.transpose());
  PRINT_VAR(C_expected.transpose());

  PRINT_VAR(link1_node_index);
  model.CalcPositionKinematicsCache(*context, &pc);
  PRINT_VARn(pc.get_mutable_X_WB(link1_node_index).matrix());
  PRINT_VARn(acrobot_benchmark.CalcLink1PoseInWorldFrame(theta1).matrix());

}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake
