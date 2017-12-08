#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/monolithic_pick_and_place_system.h"

#include <set>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

using manipulation::planner::InterpolatorType;
using robotlocomotion::robot_plan_t;
using systems::AbstractValue;
using systems::ConstantValueSource;
using systems::ZeroOrderHold;

MonolithicPickAndPlaceSystem::MonolithicPickAndPlaceSystem(
    const pick_and_place::SimulatedPlantConfiguration& plant_configuration,
    const pick_and_place::OptitrackConfiguration& optitrack_configuration,
    const std::vector<pick_and_place::PlannerConfiguration>&
        planner_configurations,
    bool single_move) {
  systems::DiagramBuilder<double> builder;

  // Add the plant.
  plant_ = builder.AddSystem<pick_and_place::LcmPlant>(plant_configuration,
                                                       optitrack_configuration);

  // Add blocks to publish contact results.
  auto contact_viz =
      builder.AddSystem<systems::ContactResultsToLcmSystem<double>>(
          plant_->get_tree());
  builder.Connect(plant_->get_output_port_contact_results(),
                  contact_viz->get_input_port(0));

  // Export outputs for visualization
  output_port_contact_results_ =
      builder.ExportOutput(contact_viz->get_output_port(0));
  output_port_plant_state_ =
      builder.ExportOutput(plant_->get_output_port_plant_state());

  // Loop over arms to add plan interpolators
  const int num_robots(plant_configuration.robot_models.size());
  for (int i = 0; i < num_robots; ++i) {
    // Add plan interpolator.
    auto plan_interpolator = builder.AddSystem<LcmPlanInterpolator>(
        FindResourceOrThrow(plant_configuration.robot_models[i]),
        InterpolatorType::Cubic);
    plan_interpolators_.push_back(plan_interpolator);
    // Add a zero-order hold between the plant and the interpolator
    auto iiwa_status_zoh = builder.AddSystem<ZeroOrderHold<double>>(
        kIiwaLcmStatusPeriod, systems::Value<lcmt_iiwa_status>());
    // Connect plan interpolator, and plant.
    // Plant --> plan interpolator
    builder.Connect(plant_->get_output_port_iiwa_status(i),
                    iiwa_status_zoh->get_input_port());
    builder.Connect(iiwa_status_zoh->get_output_port(),
                    plan_interpolator->get_input_port_iiwa_status());
    // plan interpolator --> plant
    builder.Connect(plan_interpolator->get_output_port_iiwa_command(),
                    plant_->get_input_port_iiwa_command(i));
  }

  // Loop over planner configurations to add planners
  const int num_planners(planner_configurations.size());
  DRAKE_THROW_UNLESS(num_planners >= 1);
  std::set<int> robots_with_planners_indices;
  for (int i = 0; i < num_planners; ++i) {
    // Add planner.
    auto planner = builder.AddSystem<pick_and_place::LcmPlanner>(
        planner_configurations[i], optitrack_configuration, single_move);
    planners_.push_back(planner);
    // Connect planner, plan interpolator, and plant.
    const int robot_index = planner_configurations[i].robot_index;
    robots_with_planners_indices.insert(robot_index);
    // Plant --> Planner
    builder.Connect(plant_->get_output_port_wsg_status(robot_index),
                    planner->get_input_port_wsg_status());
    builder.Connect(plant_->get_output_port_iiwa_status(robot_index),
                    planner->get_input_port_iiwa_status());
    builder.Connect(plant_->get_output_port_optitrack_frame(),
                    planner->get_input_port_optitrack_message());
    // Planner --> Plant
    builder.Connect(planner->get_output_port_wsg_command(),
                    plant_->get_input_port_wsg_command(robot_index));
    // Planner --> Plan Interpolator
    builder.Connect(
        planner->get_output_port_iiwa_plan(),
        plan_interpolators_[robot_index]->get_input_port_iiwa_plan());
  }

  // Loop over arms again to add constant sources for the robots not connected
  // to planners.
  for (int i = 0; i < num_robots; ++i) {
    if (robots_with_planners_indices.count(i) == 0) {
      auto wsg_command_source = builder.AddSystem<ConstantValueSource<double>>(
          AbstractValue::Make(lcmt_schunk_wsg_command()));
      builder.Connect(wsg_command_source->get_output_port(0),
                      plant_->get_input_port_wsg_command(i));
      auto plan_interpolator_source =
          builder.AddSystem<ConstantValueSource<double>>(
              AbstractValue::Make(robot_plan_t()));
      builder.Connect(plan_interpolator_source->get_output_port(0),
                      plan_interpolators_[i]->get_input_port_iiwa_plan());
    }
  }

  builder.BuildInto(this);
}

bool MonolithicPickAndPlaceSystem::is_done(
    const systems::Context<double>& context) const {
  for (const auto& planner : planners_) {
    if (planner->state(this->GetSubsystemContext(*planner, context)) !=
        pick_and_place::PickAndPlaceState::kDone) {
      return false;
    }
  }
  return true;
}

void MonolithicPickAndPlaceSystem::Initialize(
    systems::Context<double>* context) {
  for (auto& plan_interpolator : plan_interpolators_) {
    const VectorX<double> q0{
        VectorX<double>::Zero(plan_interpolator->num_joints())};
    auto& plan_interpolator_context =
        this->GetMutableSubsystemContext(*plan_interpolator, context);
    plan_interpolator->Initialize(context->get_time(), q0,
                                  &plan_interpolator_context);
  }
}

const pick_and_place::WorldState& MonolithicPickAndPlaceSystem::world_state(
    const systems::Context<double>& context, int index) const {
  DRAKE_DEMAND(0 <= index && index < static_cast<int>(planners_.size()));
  return this->planners_[index]->world_state(
      this->GetSubsystemContext(*planners_[index], context));
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
