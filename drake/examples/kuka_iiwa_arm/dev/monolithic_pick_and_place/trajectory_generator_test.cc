/**
 *
 * @file This is a demo of the functionality of the
 * IiwaStateFeedbackPlanSource along with the
 * SchunkWsgTrajectoryGenerator.
 *
 */

#include <gflags/gflags.h>
#include "robotlocomotion/robot_plan_t.hpp"
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/demo_diagram_builder.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/iiwa_state_feedback_plan.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

using robotlocomotion::robot_plan_t;

namespace drake {
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::DiagramBuilder;

namespace examples {
using schunk_wsg::SchunkWsgTrajectoryGenerator;

namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

class PlanSourceTester : public systems::LeafSystem<double> {
 public:
  PlanSourceTester(const RigidBodyTreed& iiwa_tree,
                   const std::vector<double>& time,
                   const std::vector<Eigen::VectorXd>& q,
                   const std::vector<double>& wsg_positions)
      : kIiwaTree(iiwa_tree),
        kPlanTime(time),
        kPlanIiwaJointAngles(q),
        kPlanWsgPositions(wsg_positions),
        kNumPoints(time.size()),
        output_port_iiwa_plan_(DeclareAbstractOutputPort().get_index()),
        output_port_wsg_plan_(DeclareAbstractOutputPort().get_index()) {
  }

  const systems::OutputPortDescriptor<double>& get_output_port_iiwa_action()
      const {
    return this->get_output_port(output_port_iiwa_plan_);
  }

  const systems::OutputPortDescriptor<double>& get_output_port_wsg_action()
      const {
    return this->get_output_port(output_port_wsg_plan_);
  }

 protected:
  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override {
    std::unique_ptr<systems::AbstractValue> return_value;
    std::unique_ptr<systems::AbstractValue> return_val;

    /* allocate outputs for IiwaStateFeedbackPlanSource and
     * SchunkWsgTrajectoryGenerator */
    if (descriptor.get_index() == output_port_iiwa_plan_) {
      return_val = systems::AbstractValue::Make<robot_plan_t>(robot_plan_t());
    } else if (descriptor.get_index() == output_port_wsg_plan_) {
      lcmt_schunk_wsg_command default_command;
      return_val = systems::AbstractValue::Make<lcmt_schunk_wsg_command>(
          default_command);
    }
    return return_val;
  }

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override {
    robot_plan_t& iiwa_action_output =
        output->GetMutableData(output_port_iiwa_plan_)
            ->GetMutableValue<robot_plan_t>();
    lcmt_schunk_wsg_command& wsg_action_output =
        output->GetMutableData(output_port_wsg_plan_)
            ->GetMutableValue<lcmt_schunk_wsg_command>();
    static bool iiwa_plan_started = false, wsg_plan_started = false;
    double time = context.get_time();
    if (time >= kPlanTime.at(0)) {
      if (!iiwa_plan_started) {
        iiwa_plan_started = true;
        drake::log()->info("Starting Robot(IIWA) plan\n");
      }

      const unsigned int kPlanSize = kPlanIiwaJointAngles.size();
      std::vector<int> info(kPlanSize, 1);
      MatrixX<double> q_mat(kPlanIiwaJointAngles.front().size(), kPlanSize);

      for (size_t i = 0; i < kPlanSize; ++i) q_mat.col(i) =
                                                 kPlanIiwaJointAngles[i];
      iiwa_action_output = EncodeKeyFrames(kIiwaTree, kPlanTime, info, q_mat);

      if (!wsg_plan_started) {
        wsg_plan_started = true;
        drake::log()->info("Starting Gripper(WSG) plan");
      }

      wsg_action_output.target_position_mm =
          kPlanWsgPositions[kPlanTime.size()];
      for (unsigned int i = 0; i < kPlanTime.size() - 1; ++i) {
        if (time >= kPlanTime.at(i) && time < kPlanTime.at(i + 1)) {
          wsg_action_output.target_position_mm = kPlanWsgPositions[i];
        }
      }
      wsg_action_output.utime = static_cast<uint64_t>(time * 1e6);
    }
  }

 private:
  const RigidBodyTreed& kIiwaTree;
  const std::vector<double> kPlanTime;
  const std::vector<Eigen::VectorXd> kPlanIiwaJointAngles;
  const std::vector<double> kPlanWsgPositions;
  const unsigned int kNumPoints{0};
  int output_port_iiwa_plan_{-1};
  int output_port_wsg_plan_{-1};
};

int DoMain(void) {
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  ModelInstanceInfo<double> iiwa_instance, wsg_instance, box_instance;

  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant<double>(&iiwa_instance, &wsg_instance, &box_instance);

  auto plant_ =
      builder.template AddSystem<IiwaAndWsgPlantWithStateEstimator<double>>(
          std::move(model_ptr), iiwa_instance, wsg_instance, box_instance);

  auto drake_visualizer_ = builder.template AddSystem<DrakeVisualizer>(
      plant_->get_plant().get_rigid_body_tree(), &lcm);

  builder.Connect(plant_->get_plant_output_port(),
                  drake_visualizer_->get_input_port(0));

  auto iiwa_zero_acceleration_source_ =
      builder.template AddSystem<systems::ConstantVectorSource>(
          Eigen::VectorXd::Zero(7));
  builder.Connect(iiwa_zero_acceleration_source_->get_output_port(),
                  plant_->get_iiwa_acceleration_input_port());

  auto iiwa_plan_source_ =
      builder.template AddSystem<IiwaStateFeedbackPlanSource>(
          drake::GetDrakePath() + kIiwaUrdf, 0.01);
  builder.Connect(plant_->get_iiwa_state_port(),
                  iiwa_plan_source_->get_input_port_state());
  builder.Connect(iiwa_plan_source_->get_output_port_trajectory(),
                  plant_->get_iiwa_state_input_port());

  auto wsg_trajectory_generator_ =
      builder.template AddSystem<SchunkWsgTrajectoryGenerator>(
          plant_->get_wsg_state_port().size(), 0);
  builder.Connect(plant_->get_wsg_state_port(),
                  wsg_trajectory_generator_->get_state_input_port());
  builder.Connect(wsg_trajectory_generator_->get_output_port(0),
                  plant_->get_wsg_input_port());

  auto iiwa_base_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      kRobotBase, Eigen::Vector3d::Zero());

  RigidBodyTree<double> iiwa;
  parsers::urdf::AddModelInstanceFromUrdfFile(drake::GetDrakePath() + kIiwaUrdf,
                                              multibody::joints::kFixed,
                                              iiwa_base_frame, &iiwa);

  double kStartTime = 0.5;
  std::vector<double> t, q_wsg;
  t.push_back(kStartTime);
  t.push_back(kStartTime + 0.5);
  t.push_back(kStartTime + 1.0);
  t.push_back(kStartTime + 1.5);

  std::vector<Eigen::VectorXd> q_iiwa;
  q_iiwa.push_back(Eigen::VectorXd::Constant(7, 0.5) -
                   0.5 * Eigen::VectorXd::Random(7));
  q_iiwa.push_back(Eigen::VectorXd::Zero(7));
  q_iiwa.push_back(Eigen::VectorXd::Constant(7, -0.5) +
                   0.5 * Eigen::VectorXd::Random(7));
  q_iiwa.push_back(Eigen::VectorXd::Zero(7));

  q_wsg.push_back(110);
  q_wsg.push_back(0);
  q_wsg.push_back(110);
  q_wsg.push_back(0);

  auto trajectory_generator_tester =
      builder.template AddSystem<PlanSourceTester>(iiwa, t, q_iiwa, q_wsg);

  builder.Connect(trajectory_generator_tester->get_output_port_iiwa_action(),
                  iiwa_plan_source_->get_input_port_plan());
  builder.Connect(trajectory_generator_tester->get_output_port_wsg_action(),
                  wsg_trajectory_generator_->get_command_input_port());

  std::cout << "Finished diagram connections. Starting simulation.\n";

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);
  simulator.Initialize();

  simulator.StepTo(6.0);

  std::cout << "Demo completed.\n";

  return 0;
}

}  // namespace
}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, const char* argv[]) {
  drake::examples::kuka_iiwa_arm::pick_and_place::DoMain();
  return 0;
}
