#include <gflags/gflags.h>
#include <memory>
#include "robotlocomotion/robot_plan_t.hpp"
#include <string>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/iiwa_state_feedback_plan.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/demo_diagram_builder.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_common.h"

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

class TrajectoryGeneratorTester : public systems::LeafSystem<double> {
 public:
  TrajectoryGeneratorTester(const RigidBodyTreed& iiwa_tree,
                            const std::vector<double>& time,
                            const std::vector<Eigen::VectorXd>& q,
                            const std::vector<double>& wsg_positions)
      : iiwa_tree_(iiwa_tree), time_(time), q_(q), wsg_positions_(wsg_positions),
        num_points_(time.size()) {
    output_port_iiwa_plan_ = DeclareAbstractOutputPort().get_index();
    output_port_wsg_plan_ = DeclareAbstractOutputPort().get_index();
//    this->DeclarePeriodicUnrestrictedUpdate(0.5, 0);
  }

  const systems::OutputPortDescriptor<double>& get_output_port_iiwa_action() const {
    return this->get_output_port(output_port_iiwa_plan_);
  }

  const systems::OutputPortDescriptor<double>& get_output_port_wsg_action() const {
    return this->get_output_port(output_port_wsg_plan_);
  }

 protected:
  /// LeafSystem override.
  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override {
    std::unique_ptr<systems::AbstractValue> return_value;
    std::unique_ptr<systems::AbstractValue> return_val;
    /* allocate iiwa action and wsg output port */
    if (descriptor.get_index() == output_port_iiwa_plan_) {
      return_val = systems::AbstractValue::Make<robot_plan_t>(
          robot_plan_t());
    } else if(descriptor.get_index() == output_port_wsg_plan_) {
      lcmt_schunk_wsg_command default_command;
      return_val = systems::AbstractValue::Make<lcmt_schunk_wsg_command>(
          default_command);
    }
    return return_val;
  }

  /// LeafSystem override.
  void DoCalcOutput(const systems::Context<double> &context,
                    systems::SystemOutput<double> *output) const override {

    std::cout<<"Lalas\n";
    robot_plan_t& iiwa_action_output =
        output->GetMutableData(output_port_iiwa_plan_)
            ->GetMutableValue<robot_plan_t>();
    lcmt_schunk_wsg_command& wsg_action_output =
        output->GetMutableData(output_port_wsg_plan_)->
            GetMutableValue<lcmt_schunk_wsg_command>();
    static bool iiwa_first_time = false, wsg_first_time = false;
    double time = context.get_time();
    if( time >= time_.at(0)){

      if(!iiwa_first_time){
        iiwa_first_time = true;
        drake::log()->info("Starting IIWA plan\n");
      }

      const unsigned long plan_size = q_.size();
      std::vector<int> info(plan_size, 1);
      MatrixX<double> q_mat(q_.front().size(),
                            plan_size);

      for (size_t i = 0; i < plan_size; ++i) q_mat.col(i) = q_[i];
      iiwa_action_output =
          EncodeKeyFrames(iiwa_tree_, time_, info, q_mat);

      if(!wsg_first_time){
       wsg_first_time = true;
        drake::log()->info("Starting WSG plan");
      }

      wsg_action_output.target_position_mm = wsg_positions_[time_.size()];
      for (unsigned int i = 0 ; i< time_.size() - 1; ++i) {
        if (time >= time_.at(i) && time < time_.at(i+1)) {
          wsg_action_output.target_position_mm = wsg_positions_[i];
          drake::log()->info("Setting WSG plan to {}", wsg_positions_[i] );
        }
      }
      wsg_action_output.utime = static_cast<uint64_t>(time * 1e6);
    }
  }
 private:
  const RigidBodyTreed& iiwa_tree_;
  const std::vector<double> time_;
  const std::vector<Eigen::VectorXd> q_;
  const std::vector<double> wsg_positions_;
  const unsigned int num_points_{0};
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

  builder.Connect(
      plant_->get_plant_output_port(),
      drake_visualizer_->get_input_port(0));

  auto iiwa_zero_acceleration_source_ = builder.template
      AddSystem<systems::ConstantVectorSource>(Eigen::VectorXd::Zero(7));
  builder.Connect(
      iiwa_zero_acceleration_source_->get_output_port(),
      plant_->
          get_iiwa_acceleration_input_port());

  auto iiwa_trajectory_generator_ = builder.template
      AddSystem<IiwaStateFeedbackTrajectoryGenerator>(
      drake::GetDrakePath() + kIiwaUrdf, 0.01);
  builder.Connect(plant_->get_iiwa_state_port(),
                  iiwa_trajectory_generator_->get_input_port_state());
  builder.Connect(iiwa_trajectory_generator_->get_output_port_trajectory(),
                  plant_->get_iiwa_state_input_port());

  auto wsg_trajectory_generator_ = builder.template
      AddSystem<SchunkWsgTrajectoryGenerator>(
      plant_->get_wsg_state_port().size(), 0);
  builder.Connect(plant_->get_wsg_state_port(),
                  wsg_trajectory_generator_->get_state_input_port());
  builder.Connect(wsg_trajectory_generator_->get_output_port(0),
                  plant_->get_wsg_input_port());

  auto iiwa_base_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      kRobotBase, Eigen::Vector3d::Zero());

  RigidBodyTree<double> iiwa;
  parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + kIiwaUrdf, multibody::joints::kFixed, iiwa_base_frame, &iiwa);

  double kStartTime = 0.5;
  std::vector<double> t, q_wsg;
  t.push_back(kStartTime);
  t.push_back(kStartTime + 0.5);
  t.push_back(kStartTime + 1.0);
  t.push_back(kStartTime + 1.5);
  std::vector<Eigen::VectorXd> q_iiwa;
  q_iiwa.push_back(Eigen::VectorXd::Constant(7, 0.5) - 0.5 * Eigen::VectorXd::Random(7));
  q_iiwa.push_back(Eigen::VectorXd::Zero(7));
  q_iiwa.push_back(Eigen::VectorXd::Constant(7, -0.5) + 0.5 * Eigen::VectorXd::Random(7));
  q_iiwa.push_back(Eigen::VectorXd::Zero(7));
  q_wsg.push_back(110);
  q_wsg.push_back(0);
  q_wsg.push_back(110);
  q_wsg.push_back(0);

  auto trajectory_generator_tester = builder.template AddSystem<TrajectoryGeneratorTester>(
      iiwa, t, q_iiwa, q_wsg);

  builder.Connect(trajectory_generator_tester->get_output_port_iiwa_action(),
                  iiwa_trajectory_generator_->get_input_port_plan());
  builder.Connect(trajectory_generator_tester->get_output_port_wsg_action(),
                  wsg_trajectory_generator_->get_command_input_port());

  std::cout<<"Finished diagram connections. Starting simulation.\n";

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);
  simulator.Initialize();

  simulator.StepTo(0.1);

  std::cout<<"Demo completed.\n";

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
