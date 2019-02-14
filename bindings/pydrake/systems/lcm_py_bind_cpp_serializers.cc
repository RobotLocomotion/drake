#include "drake/bindings/pydrake/systems/lcm_py_bind_cpp_serializers.h"

#include "robotlocomotion/image_array_t.hpp"
#include "robotlocomotion/quaternion_t.hpp"

#include "drake/bindings/pydrake/systems/lcm_pybind.h"
#include "drake/lcmt_contact_results_for_viz.hpp"

namespace drake {
namespace pydrake {
namespace pysystems {
namespace pylcm {

void BindCppSerializers() {
  // N.B. At least one type should be bound to ensure the template is defined.
  // N.B. These should be placed in the same order has the headers.
  BindCppSerializer<robotlocomotion::image_array_t>("robotlocomotion");
  BindCppSerializer<robotlocomotion::quaternion_t>("robotlocomotion");
  BindCppSerializer<drake::lcmt_contact_results_for_viz>("drake");
}

}  // namespace pylcm
}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
