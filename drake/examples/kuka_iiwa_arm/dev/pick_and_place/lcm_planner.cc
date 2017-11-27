#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/lcm_planner.h"

#include "optitrack/optitrack_frame_t.hpp"

#include "drake/manipulation/perception/optitrack_pose_extractor.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

using manipulation::perception::OptitrackPoseExtractor;
using systems::DiagramBuilder;
using systems::PassThrough;
using systems::Value;

namespace {

class OptitrackTranslatorSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackTranslatorSystem);

  OptitrackTranslatorSystem() {
    this->DeclareAbstractInputPort(
        systems::Value<Isometry3<double>>(Isometry3<double>::Identity()));
    this->DeclareAbstractOutputPort(bot_core::robot_state_t(),
                                    &OptitrackTranslatorSystem::OutputPose);
  }

 private:
  void OutputPose(const systems::Context<double>& context,
                  bot_core::robot_state_t* out) const {
    const Isometry3<double>& in = this->EvalAbstractInput(context, 0)
                                      ->template GetValue<Isometry3<double>>();

    out->utime = 0;
    EncodePose(in, out->pose);
  }
};
}  // namespace

LcmPlanner::LcmPlanner(
    const pick_and_place::PlannerConfiguration& planner_configuration,
    const pick_and_place::OptitrackConfiguration optitrack_configuration,
    bool single_move) {
  DiagramBuilder<double> builder;

  state_machine_ = builder.AddSystem<PickAndPlaceStateMachineSystem>(
      planner_configuration, single_move);

  // Export input ports for WSG status message.
  input_port_wsg_status_ =
      builder.ExportInput(state_machine_->get_input_port_wsg_status());

  // The Optitrack message needs to be passed to multiple OptitrackPoseExtractor
  // blocks, this pass-through block allows that.
  auto optitrack_message_passthrough = builder.AddSystem<PassThrough<double>>(
      Value<optitrack::optitrack_frame_t>());

  // Export input port for the Optitrack message.
  input_port_optitrack_message_ =
      builder.ExportInput(optitrack_message_passthrough->get_input_port());

  const Isometry3<double>& X_WO{Isometry3<double>::Identity()};
  const double kOptitrackLcmStatusPeriod{1.0 / 120.0};

  // Connect blocks for target.
  auto optitrack_target_pose_extractor =
      builder.AddSystem<OptitrackPoseExtractor>(
          optitrack_configuration
              .object_optitrack_info[planner_configuration.target_index]
              .id,
          X_WO, kOptitrackLcmStatusPeriod);
  optitrack_target_pose_extractor->set_name("Optitrack target pose extractor");

  auto optitrack_target_translator =
      builder.AddSystem<OptitrackTranslatorSystem>();
  optitrack_target_translator->set_name("Optitrack target translator");

  builder.Connect(optitrack_message_passthrough->get_output_port(),
                  optitrack_target_pose_extractor->get_input_port(0));
  builder.Connect(optitrack_target_pose_extractor->get_output_port(0),
                  optitrack_target_translator->get_input_port(0));
  builder.Connect(optitrack_target_translator->get_output_port(0),
                  state_machine_->get_input_port_box_state());

  // Connect Optitrack blocks for tables.
  for (int i = 0; i < static_cast<int>(
                          optitrack_configuration.table_optitrack_info.size());
       ++i) {
    auto optitrack_table_pose_extractor =
        builder.AddSystem<manipulation::perception::OptitrackPoseExtractor>(
            optitrack_configuration.table_optitrack_info[i].id, X_WO,
            kOptitrackLcmStatusPeriod);
    optitrack_table_pose_extractor->set_name(
        "Optitrack table " + std::to_string(i) + "  pose extractor");
    builder.Connect(optitrack_message_passthrough->get_output_port(),
                    optitrack_table_pose_extractor->get_input_port(0));
    builder.Connect(optitrack_table_pose_extractor->get_output_port(0),
                    state_machine_->get_input_port_table_state(i));
  }

  // Connect Optitrack blocks for IIWA base.
  auto optitrack_iiwa_base_pose_extractor =
      builder.AddSystem<OptitrackPoseExtractor>(
          optitrack_configuration
              .robot_base_optitrack_info[planner_configuration.robot_index]
              .id,
          X_WO, kOptitrackLcmStatusPeriod);
  optitrack_iiwa_base_pose_extractor->set_name(
      "Optitrack IIWA base pose extractor");

  builder.Connect(optitrack_message_passthrough->get_output_port(),
                  optitrack_iiwa_base_pose_extractor->get_input_port(0));
  builder.Connect(optitrack_iiwa_base_pose_extractor->get_output_port(0),
                  state_machine_->get_input_port_iiwa_base_pose());

  // Export input port for IIWA status message.
  input_port_iiwa_status_ =
      builder.ExportInput(state_machine_->get_input_port_iiwa_state());

  // Export output ports.
  output_port_iiwa_plan_ =
      builder.ExportOutput(state_machine_->get_output_port_iiwa_plan());
  output_port_wsg_command_ =
      builder.ExportOutput(state_machine_->get_output_port_wsg_command());

  // Build the system.
  builder.BuildInto(this);
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
