#include "drake/bindings/pydrake/systems/lcm_py_bind_cpp_serializers.h"

#include "drake/bindings/pydrake/systems/lcm_pybind.h"
#include "drake/experimental_lcmt_deformable_tri.hpp"
#include "drake/experimental_lcmt_deformable_tri_mesh_init.hpp"
#include "drake/experimental_lcmt_deformable_tri_mesh_update.hpp"
#include "drake/experimental_lcmt_deformable_tri_meshes_init.hpp"
#include "drake/experimental_lcmt_deformable_tri_meshes_update.hpp"
#include "drake/lcmt_acrobot_u.hpp"
#include "drake/lcmt_acrobot_x.hpp"
#include "drake/lcmt_acrobot_y.hpp"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"
#include "drake/lcmt_call_python.hpp"
#include "drake/lcmt_call_python_data.hpp"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/lcmt_force_torque.hpp"
#include "drake/lcmt_header.hpp"
#include "drake/lcmt_hydroelastic_contact_surface_for_viz.hpp"
#include "drake/lcmt_hydroelastic_quadrature_per_point_data_for_viz.hpp"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_iiwa_status_telemetry.hpp"
#include "drake/lcmt_image.hpp"
#include "drake/lcmt_image_array.hpp"
#include "drake/lcmt_jaco_command.hpp"
#include "drake/lcmt_jaco_status.hpp"
#include "drake/lcmt_panda_command.hpp"
#include "drake/lcmt_panda_status.hpp"
#include "drake/lcmt_planar_gripper_command.hpp"
#include "drake/lcmt_planar_gripper_finger_command.hpp"
#include "drake/lcmt_planar_gripper_finger_face_assignment.hpp"
#include "drake/lcmt_planar_gripper_finger_face_assignments.hpp"
#include "drake/lcmt_planar_gripper_finger_status.hpp"
#include "drake/lcmt_planar_gripper_status.hpp"
#include "drake/lcmt_planar_manipuland_status.hpp"
#include "drake/lcmt_planar_plant_state.hpp"
#include "drake/lcmt_point.hpp"
#include "drake/lcmt_point_cloud.hpp"
#include "drake/lcmt_point_cloud_field.hpp"
#include "drake/lcmt_point_pair_contact_info_for_viz.hpp"
#include "drake/lcmt_quaternion.hpp"
#include "drake/lcmt_robot_plan.hpp"
#include "drake/lcmt_robot_state.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmt_scope.hpp"
#include "drake/lcmt_viewer_command.hpp"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_link_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"

namespace drake {
namespace pydrake {
namespace pysystems {
namespace pylcm {

void BindCppSerializers() {
  // N.B. These should be placed in the same order as the headers.
  BindCppSerializer<drake::experimental_lcmt_deformable_tri>("drake");
  BindCppSerializer<drake::experimental_lcmt_deformable_tri_mesh_init>("drake");
  BindCppSerializer<drake::experimental_lcmt_deformable_tri_mesh_update>(
      "drake");
  BindCppSerializer<drake::experimental_lcmt_deformable_tri_meshes_init>(
      "drake");
  BindCppSerializer<drake::experimental_lcmt_deformable_tri_meshes_update>(
      "drake");
  BindCppSerializer<drake::lcmt_acrobot_u>("drake");
  BindCppSerializer<drake::lcmt_acrobot_x>("drake");
  BindCppSerializer<drake::lcmt_acrobot_y>("drake");
  BindCppSerializer<drake::lcmt_allegro_command>("drake");
  BindCppSerializer<drake::lcmt_allegro_status>("drake");
  BindCppSerializer<drake::lcmt_call_python>("drake");
  BindCppSerializer<drake::lcmt_call_python_data>("drake");
  BindCppSerializer<drake::lcmt_contact_results_for_viz>("drake");
  BindCppSerializer<drake::lcmt_drake_signal>("drake");
  BindCppSerializer<drake::lcmt_force_torque>("drake");
  BindCppSerializer<drake::lcmt_header>("drake");
  BindCppSerializer<drake::lcmt_hydroelastic_contact_surface_for_viz>("drake");
  BindCppSerializer<drake::lcmt_hydroelastic_quadrature_per_point_data_for_viz>(
      "drake");
  BindCppSerializer<drake::lcmt_iiwa_command>("drake");
  BindCppSerializer<drake::lcmt_iiwa_status>("drake");
  BindCppSerializer<drake::lcmt_iiwa_status_telemetry>("drake");
  BindCppSerializer<drake::lcmt_image>("drake");
  BindCppSerializer<drake::lcmt_image_array>("drake");
  BindCppSerializer<drake::lcmt_jaco_command>("drake");
  BindCppSerializer<drake::lcmt_jaco_status>("drake");
  BindCppSerializer<drake::lcmt_panda_command>("drake");
  BindCppSerializer<drake::lcmt_panda_status>("drake");
  BindCppSerializer<drake::lcmt_planar_gripper_command>("drake");
  BindCppSerializer<drake::lcmt_planar_gripper_finger_command>("drake");
  BindCppSerializer<drake::lcmt_planar_gripper_finger_face_assignment>("drake");
  BindCppSerializer<drake::lcmt_planar_gripper_finger_face_assignments>(
      "drake");
  BindCppSerializer<drake::lcmt_planar_gripper_finger_status>("drake");
  BindCppSerializer<drake::lcmt_planar_gripper_status>("drake");
  BindCppSerializer<drake::lcmt_planar_manipuland_status>("drake");
  BindCppSerializer<drake::lcmt_planar_plant_state>("drake");
  BindCppSerializer<drake::lcmt_point>("drake");
  BindCppSerializer<drake::lcmt_point_cloud>("drake");
  BindCppSerializer<drake::lcmt_point_cloud_field>("drake");
  BindCppSerializer<drake::lcmt_point_pair_contact_info_for_viz>("drake");
  BindCppSerializer<drake::lcmt_quaternion>("drake");
  BindCppSerializer<drake::lcmt_robot_plan>("drake");
  BindCppSerializer<drake::lcmt_robot_state>("drake");
  BindCppSerializer<drake::lcmt_schunk_wsg_command>("drake");
  BindCppSerializer<drake::lcmt_schunk_wsg_status>("drake");
  BindCppSerializer<drake::lcmt_scope>("drake");
  BindCppSerializer<drake::lcmt_viewer_command>("drake");
  BindCppSerializer<drake::lcmt_viewer_draw>("drake");
  BindCppSerializer<drake::lcmt_viewer_geometry_data>("drake");
  BindCppSerializer<drake::lcmt_viewer_link_data>("drake");
  BindCppSerializer<drake::lcmt_viewer_load_robot>("drake");
}

}  // namespace pylcm
}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
