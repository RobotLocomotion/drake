#include "drake/automotive/single_lane_ego_and_agent.h"

#include "drake/common/drake_export.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace automotive {

template <typename T>
SingleLaneEgoAndAgent<T>::SingleLaneEgoAndAgent(const T& v_0)
  : systems::Diagram<T>() {

  // Keep the reference velocity nonnegative.
  DRAKE_ASSERT(v_0 >= 0);

  systems::DiagramBuilder<T> builder;
  ego_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());
  agent_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());
  planner_ = builder.AddSystem(std::make_unique<IdmPlanner<T>>(
                                              v_0 /* desired velocity */));

  builder.ExportInput(ego_car_->get_input_port());
  builder.ExportInput(agent_car_->get_input_port());
  builder.ExportInput(planner_->get_input_port(0));
  builder.ExportInput(planner_->get_input_port(1));

  builder.ExportOutput(ego_car_->get_output_port());
  builder.ExportOutput(agent_car_->get_output_port());
  builder.ExportOutput(planner_->get_output_port(0));

  builder.Connect(*planner_, *ego_car_);
  builder.Connect(ego_car_->get_output_port(),
                  planner_->get_input_port(0));
  builder.Connect(agent_car_->get_output_port(),
                  planner_->get_input_port(1));
  builder.BuildInto(this);
}

template class DRAKE_EXPORT SingleLaneEgoAndAgent<double>;
template class DRAKE_EXPORT SingleLaneEgoAndAgent<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
