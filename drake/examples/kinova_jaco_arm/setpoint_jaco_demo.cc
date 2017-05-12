/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to reach to a
/// position in space and then repeat this reaching task with a different joint
/// configuration constraint.

#include <iostream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kinova_jaco_arm/iiwa_common.h"
#include "drake/examples/kinova_jaco_arm/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
//#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, 4, "Number of seconds to simulate.");

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;
using std::make_unique;
using std::move;
using std::string;
using std::unique_ptr;

namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace {

const char kUrdfPath[] = "/manipulation/models/jaco_description/urdf/"
            "j2n6s300.urdf";

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  systems::RigidBodyPlant<double>* plant = nullptr;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  CreateTreedFromFixedModelAtPose(kUrdfPath, tree.get());

  // (Rick) may just want to ignore MakePlan here, and do something in joint space
  //std::unique_ptr<PiecewisePolynomialTrajectory> traj = MakePlan();

  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;
  // Adds a plant
  plant = builder.AddPlant(std::move(tree));
  builder.AddVisualizer(&lcm);

  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  auto controller =
      builder.AddController<systems::InverseDynamicsController<double>>(
          RigidBodyTreeConstants::kFirstNonWorldModelInstanceId,
          GetDrakePath() + kUrdfPath, nullptr, iiwa_kp, iiwa_ki, iiwa_kd,
          false /* no feedforward acceleration */);

  // Adds a trajectory source for desired state.
  systems::DiagramBuilder<double>* diagram_builder =
      builder.get_mutable_builder();

//  auto traj_src =
//      diagram_builder->template AddSystem<systems::TrajectorySource<double>>(
//          *traj, 1 /* outputs q + v */);
//  traj_src->set_name("trajectory_source");

  VectorXd const_pos = VectorXd::Zero(18);
  //const_pos(1)=3; const_pos(2)=3;

  systems::ConstantVectorSource<double>* const_src = diagram_builder->AddSystem<systems::ConstantVectorSource<double>>(
          const_pos);

  const_src->set_name("constant_source");
  diagram_builder->Connect(const_src->get_output_port(),
                  controller->get_input_port_desired_state());

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

//  systems::Context<double>* jaco_context =
//          diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
//                                              plant);

//  //  Set the initial conditions
//  systems::VectorBase<double>* x0 = jaco_context->get_mutable_continuous_state_vector();
//  x0->SetAtIndex(0,0);
//  x0->SetAtIndex(1,1.5);
//  x0->SetAtIndex(2,1.5);

  simulator.Initialize();
  simulator.set_target_realtime_rate(0.5);

  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kinova_jaco_arm::DoMain();
}
