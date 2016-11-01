#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/plants/RigidBodySystem.h"

using Eigen::VectorXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace plants {
namespace rigid_body_plant {
namespace test {
namespace {

template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

// Tests RigidBodyPlant<T>::EvalTimeDerivatives() for a Kuka arm model.
// The test is performed by comparing against the results obtained with an RBS1
// model of the same Kuka arm.
// "RBS1" is an abbreviation referring to version 1.0 of RigidBodySystem as
// implemented in:
// drake/systems/plants/RigidBodySystem.h.
// Within this test we will refer to the implementation under test as RBS2
// (Rigid Body System 2.0). The naming conventions for the variables in this
// test are lower case versions of these acronyms.
GTEST_TEST(RigidBodySystemTest, CompareWithRBS1Dynamics) {
  //////////////////////////////////////////////////////////////////////////////
  // Instantiates a RigidBodySystem (System 1.0) model of the Kuka arm.
  //////////////////////////////////////////////////////////////////////////////
  auto rbs1 = make_unique<RigidBodySystem>();

  // Adds a URDF to the rigid body system. This URDF contains only fixed joints
  // and is attached to the world via a fixed joint. Thus, everything in the
  // URDF becomes part of the world.
  rbs1->AddModelInstanceFromFile(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      drake::systems::plants::joints::kFixed);

  //////////////////////////////////////////////////////////////////////////////
  // Instantiates a RigidBodyPlant (System 2.0) model of the Kuka arm.
  //////////////////////////////////////////////////////////////////////////////
  auto tree = make_unique<RigidBodyTree>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      drake::systems::plants::joints::kFixed, nullptr /* weld to frame */,
      tree.get());

  // Instantiates a RigidBodyPlant (System 2.0) from an MBD model of the world.
  auto rbs2 = make_unique<RigidBodyPlant<double>>(move(tree));

  auto context = rbs2->CreateDefaultContext();
  auto output = rbs2->AllocateOutput(*context);
  auto derivatives = rbs2->AllocateTimeDerivatives();

  //////////////////////////////////////////////////////////////////////////////
  // Sets the initial condition of both systems to be the same.
  //////////////////////////////////////////////////////////////////////////////
  // For rbs1:
  // Obtains an initial state of the simulation.
  VectorXd x0 = VectorXd::Zero(rbs1->getNumStates());
  x0.head(rbs1->get_num_positions()) =
      rbs1->getRigidBodyTree()->getZeroConfiguration();

  // Some non-zero velocities.
  x0.tail(rbs1->get_num_velocities()) << 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0;

  Eigen::VectorXd arbitrary_angles(rbs1->get_num_positions());
  arbitrary_angles << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;
  x0.head(rbs1->get_num_positions()) += arbitrary_angles;

  // For rbs2:
  // Zeroes the state.
  rbs2->SetZeroConfiguration(context.get());

  // Sets the state to a non-zero value matching the configuration for rbs1.
  rbs2->set_state_vector(context.get(), x0);
  VectorXd xc = context->get_continuous_state()->CopyToVector();
  ASSERT_EQ(xc, x0);

  //////////////////////////////////////////////////////////////////////////////
  // Sets the inputs (generalized forces) to be the same.
  //////////////////////////////////////////////////////////////////////////////
  // For rbs1:
  VectorX<double> u = VectorX<double>::Zero(rbs1->getNumInputs());
  u << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;

  // For rbs2:
  // Connect to a "fake" free standing input with the same values used for RBS1.
  // TODO(amcastro-tri): Connect to a ConstantVectorSource once Diagrams have
  // derivatives per #3218.
  auto input_vector = std::make_unique<BasicVector<double>>(
      rbs2->get_num_actuators());
  input_vector->set_value(u);
  context->SetInputPort(0, MakeInput(move(input_vector)));

  //////////////////////////////////////////////////////////////////////////////
  // Computes time derivatives to compare against rbs1 dynamics.
  //////////////////////////////////////////////////////////////////////////////
  auto rbs1_xdot = rbs1->dynamics(0.0, x0, u);
  rbs2->EvalTimeDerivatives(*context, derivatives.get());
  auto rbs2_xdot = derivatives->CopyToVector();

  //////////////////////////////////////////////////////////////////////////////
  // Performs the comparison.
  //////////////////////////////////////////////////////////////////////////////
  EXPECT_TRUE(rbs1->get_num_positions() == rbs2->get_num_positions());
  EXPECT_TRUE(rbs1->get_num_velocities() == rbs2->get_num_velocities());
  EXPECT_TRUE(rbs2_xdot.isApprox(rbs1_xdot));
}

}  // namespace
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake
