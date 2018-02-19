#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"

#include "drake/systems/rendering/drake_visualizer_client.h"

namespace drake {

using systems::rendering::MakeGeometryData;

namespace multibody {

template <typename T>
lcmt_viewer_load_robot CreateLoadRobotMessage(
    const RigidBodyTree<double>& tree) {
  lcmt_viewer_load_robot load_message;
  load_message.num_links = tree.get_bodies().size();
  for (const auto& body : tree.get_bodies()) {
    lcmt_viewer_link_data link;
    link.name = body->get_name();
    link.robot_num = body->get_model_instance_id();
    link.num_geom = body->get_visual_elements().size();
    for (const auto& visual_element : body->get_visual_elements()) {
      link.geom.push_back(MakeGeometryData(visual_element));
    }
    load_message.link.push_back(link);
  }
  return load_message;
}

template lcmt_viewer_load_robot CreateLoadRobotMessage<double>(
  const RigidBodyTree<double>& tree);

}  // namespace multibody
}  // namespace drake
