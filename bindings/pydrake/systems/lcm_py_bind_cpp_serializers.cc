#include "drake/bindings/pydrake/systems/lcm_py_bind_cpp_serializers.h"

#include "drake/bindings/pydrake/systems/lcm_pybind.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_image_array.hpp"
#include "drake/lcmt_quaternion.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

namespace drake {
namespace pydrake {
namespace pysystems {
namespace pylcm {

void BindCppSerializers() {
  // N.B. At least one type should be bound to ensure the template is defined.
  // N.B. These should be placed in the same order as the headers.
  BindCppSerializer<drake::lcmt_contact_results_for_viz>("pydrake.lcmtypes");
  BindCppSerializer<drake::lcmt_iiwa_command>("pydrake.lcmtypes");
  BindCppSerializer<drake::lcmt_iiwa_status>("pydrake.lcmtypes");
  BindCppSerializer<drake::lcmt_image_array>("pydrake.lcmtypes");
  BindCppSerializer<drake::lcmt_quaternion>("pydrake.lcmtypes");
  BindCppSerializer<drake::lcmt_schunk_wsg_command>("pydrake.lcmtypes");
  BindCppSerializer<drake::lcmt_schunk_wsg_status>("pydrake.lcmtypes");
}

}  // namespace pylcm
}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
