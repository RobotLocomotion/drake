#include <functional>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/benchmarks/cylinder_torque_free_analytical_solution/torque_free_cylinder_exact_solution.h"
#include "drake/multibody/multibody_tree/test/free_body_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {
namespace {

//const double kEpsilon = std::numeric_limits<double>::epsilon();

using benchmarks::cylinder_torque_free_analytical_solution::TorqueFreeCylinderExactSolution;
using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Translation3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::make_unique;
using std::unique_ptr;
using std::vector;
using systems::Context;
using systems::RungeKutta3Integrator;

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;


GTEST_TEST(RollPitchYawTest, TimeDerivatives) {
  const double kAccuracy = 1.0e-5;
  const double max_dt = 0.1;
  const double end_time = 10.0;

  Vector3d w0_WB =
      Vector3d::UnitX() + Vector3d::UnitY() + Vector3d::UnitZ();
  Vector3d p0_WBcm = Vector3d::Zero();
  Vector3d v0_WBcm = Vector3d::Zero();
  Vector3d gravity_W = Vector3d::Zero();

  TorqueFreeCylinderExactSolution benchmark_(
      Quaterniond::Identity(), w0_WB,
      p0_WBcm, v0_WBcm, gravity_W);

#if 0
  systems::DiagramBuilder<double> builder;
  auto free_body_plant =
      builder.AddSystem<FreeBodyPlant>(benchmark_.get_I(), benchmark_.get_J());
  auto diagram = builder.Build();
  (void) free_body_plant;
#endif
  FreeBodyPlant<double> free_body_plant(benchmark_.get_I(), benchmark_.get_J());

  systems::Simulator<double> simulator(free_body_plant);
  systems::Context<double>* context = simulator.get_mutable_context();

  free_body_plant.set_angular_velocity(context, w0_WB);

  simulator.Initialize();
  //const double max_step_size = dt;
  RungeKutta3Integrator<double>* integrator =
          simulator.reset_integrator<RungeKutta3Integrator<double>>(
              free_body_plant, simulator.get_mutable_context());
  integrator->set_maximum_step_size(max_dt);
  PRINT_VAR(integrator->get_fixed_step_mode());
  PRINT_VAR(integrator->supports_error_estimation());
  integrator->set_target_accuracy(kAccuracy);
  PRINT_VAR(integrator->get_target_accuracy());

  // Simulate:
  simulator.StepTo(end_time);

  // Get solution:
  Isometry3d X_WB = free_body_plant.CalcPoseInWorldFrame(*context);
  SpatialVelocity<double> V_WB =
      free_body_plant.CalcSpatialVelocityInWorldFrame(*context);
  const Vector3d& w_WB_W = V_WB.rotational();

  PRINT_VAR(integrator->get_num_steps_taken());
  PRINT_VAR(integrator->get_num_step_shrinkages_from_substep_failures());
  PRINT_VAR(integrator->get_num_step_shrinkages_from_error_control());
  PRINT_VAR(simulator.get_integrator()->get_smallest_adapted_step_size_taken());
  PRINT_VAR(simulator.get_integrator()->get_largest_step_size_taken());

  // Compute benchmark solution:
  Quaterniond quat_WB_exact;
  Vector4d quatDt_WB_exact;
  Vector3d w_WB_B_exact, wDt_WB_B_exact;
  std::tie(quat_WB_exact, quatDt_WB_exact, w_WB_B_exact, wDt_WB_B_exact) =
      benchmark_.CalculateExactRotationalSolutionNB(end_time);
  Vector4d qv; qv << quat_WB_exact.w(), quat_WB_exact.vec();
  Matrix3d R_WB_exact = math::quat2rotmat(qv);
  Vector3d w_WB_W_exact = R_WB_exact * w_WB_B_exact;

  PRINT_VAR(quat_WB_exact.coeffs().transpose());
  PRINT_VAR(quatDt_WB_exact.transpose());
  PRINT_VAR(w_WB_B_exact.transpose());
  PRINT_VAR(wDt_WB_B_exact.transpose());
  PRINT_VARn(R_WB_exact);
  PRINT_VARn(X_WB.linear());
  PRINT_VARn(X_WB.translation());

  Matrix3d R_WB = X_WB.linear();
  Vector3d p_WBcm = X_WB.translation();

  // Compare computed solution against benchmark:
  PRINT_VARn((R_WB - R_WB_exact).norm());
  PRINT_VARn(p_WBcm.norm());

  PRINT_VAR(w_WB_W_exact.transpose());
  PRINT_VAR(w_WB_W.transpose());

  PRINT_VAR((w_WB_W - w_WB_W_exact).norm());

  EXPECT_TRUE((R_WB - R_WB_exact).norm() < kAccuracy);
  EXPECT_TRUE((w_WB_W - w_WB_W_exact).norm() < kAccuracy * w0_WB.norm());

  // TODO(amcastro-tri): For the PR, verify angular momentum is conserved.
  
  // TODO(amcastro-tri): For the PR, verify total energy (kinetic) is conserved.

}

}  // namespace
}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
