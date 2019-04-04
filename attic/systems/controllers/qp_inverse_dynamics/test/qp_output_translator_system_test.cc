#include "drake/systems/controllers/qp_inverse_dynamics/qp_output_translator_system.h"

#include <memory>

#include <gtest/gtest.h>
#include "bot_core/atlas_command_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics_common.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {

// Checks that QpOutputTranslatorSystem's torque output port's value
// equals RBT.B^T * qp_output.dof_torques().
GTEST_TEST(QpOutputTranslatorTest, QpOutputTranslatorTest) {
  auto robot = std::make_unique<RigidBodyTree<double>>();
  // Use this model because the dof order and actuator order are different.
  parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow(
          "drake/examples/valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"),
      drake::multibody::joints::kQuaternion, nullptr /* weld to frame */,
      robot.get());

  auto dut = std::make_unique<QpOutputTranslatorSystem>(*robot);
  auto context = dut->CreateDefaultContext();
  auto output = dut->AllocateOutput();

  context->SetTime(3.);

  auto qp_output = std::make_unique<QpOutput>(GetDofNames(*robot));
  for (int i = 0; i < qp_output->dof_torques().size(); i++) {
    qp_output->mutable_dof_torques()[i] = i * 0.3;
  }

  auto input = AbstractValue::Make<QpOutput>(*qp_output);
  context->FixInputPort(dut->get_input_port_qp_output().get_index(),
                        std::move(input));

  dut->CalcOutput(*context, output.get());

  VectorX<double> expected_torque =
      robot->B.transpose() * qp_output->dof_torques();
  VectorX<double> dutoutput =
      output->get_vector_data(dut->get_output_port_torque().get_index())
          ->get_value();

  EXPECT_TRUE(drake::CompareMatrices(expected_torque, dutoutput, 1e-12,
                                     drake::MatrixCompareType::absolute));
}

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
