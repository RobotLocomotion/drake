#include "drake/bindings/pydrake/systems/lcm_py_bind_cpp_serializers.h"

#include "robotlocomotion/quaternion_t.hpp"
#include "drake/lcmt_contact_results_for_viz.hpp"

#include "drake/bindings/pydrake/systems/lcm_pybind.h"

namespace drake {
namespace pydrake {
namespace pysystems {
namespace pylcm {

void BindCppSerializers() {
  // N.B. At least one type should be bound to ensure the template is defined.
  BindCppSerializer<robotlocomotion::quaternion_t>("robotlocomotion");
  BindCppSerializer<drake::lcmt_contact_results_for_viz>("drake");
}

}  // namespace pylcm
}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
