#include "drake/systems/controllers/inverse_dynamics_controller.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"

namespace drake {
namespace systems {
namespace {

VectorX<double> ComputeTorque(const RigidBodyTree<double>& tree,
                              const VectorX<double>& q,
                              const VectorX<double>& v,
                              const VectorX<double>& vd_d) {
  // Compute the expected torque.
  KinematicsCache<double> cache = tree.doKinematics(q, v);
  eigen_aligned_std_unordered_map<RigidBody<double> const*,
                                  drake::TwistVector<double>>
      f_ext;

  return tree.massMatrix(cache) * vd_d + tree.dynamicsBiasTerm(cache, f_ext);
}

// Tests the computed torque from InverseDynamicsController matches hand
// derived results for the kuka iiwa arm at a given state (q, v), when
// asked to track reference state (q_r, v_r) and reference acceleration (vd_r).
GTEST_TEST(InverseDynamicsControllerTest, TestTorque) {
  auto robot = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
          "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14.urdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      robot.get());
  const int dim = robot->get_num_positions();

  // Sets pid gains.
  VectorX<double> kp(dim), ki(dim), kd(dim);
  kp << 1, 2, 3, 4, 5, 6, 7;
  ki << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  kd = kp / 2.;

  auto dut = std::make_unique<InverseDynamicsController<double>>(*robot, kp, ki,
                                                                 kd, true);
  auto context = dut->CreateDefaultContext();
  auto output = dut->AllocateOutput(*context);

  // Sets current state and reference state and acceleration values.
  VectorX<double> q(dim), v(dim), q_r(dim), v_r(dim), vd_r(dim);
  q << 0.3, 0.2, 0.1, 0, -0.1, -0.2, -0.3;
  v = q * 3;

  q_r = (q + VectorX<double>::Constant(dim, 0.1)) * 2.;
  v_r.setZero();
  vd_r << 1, 2, 3, 4, 5, 6, 7;

  // Connects inputs.
  auto state_input = std::make_unique<BasicVector<double>>(
      robot->get_num_positions() + robot->get_num_velocities());
  state_input->get_mutable_value() << q, v;

  auto reference_state_input = std::make_unique<BasicVector<double>>(
      robot->get_num_positions() + robot->get_num_velocities());
  reference_state_input->get_mutable_value() << q_r, v_r;

  auto reference_acceleration_input =
      std::make_unique<BasicVector<double>>(robot->get_num_velocities());
  reference_acceleration_input->get_mutable_value() << vd_r;

  context->FixInputPort(dut->get_input_port_estimated_state().get_index(),
                        std::move(state_input));
  context->FixInputPort(dut->get_input_port_desired_state().get_index(),
                        std::move(reference_state_input));
  context->FixInputPort(dut->get_input_port_desired_acceleration().get_index(),
                        std::move(reference_acceleration_input));

  // Sets integrated position error.
  VectorX<double> q_int(dim);
  q_int << -1, -2, -3, -4, -5, -6, -7;
  dut->set_integral_value(context.get(), q_int);

  // Computes output.
  dut->CalcOutput(*context, output.get());

  // The results should equal to this.
  VectorX<double> vd_d = (kp.array() * (q_r - q).array()).matrix() +
                         (kd.array() * (v_r - v).array()).matrix() +
                         (ki.array() * q_int.array()).matrix() + vd_r;

  VectorX<double> expected_torque = ComputeTorque(*robot, q, v, vd_d);

  // Checks the expected and computed gravity torque.
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
                              1e-10, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace systems
}  // namespace drake
