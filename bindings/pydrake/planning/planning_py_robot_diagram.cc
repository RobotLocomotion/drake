#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/robot_diagram.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningRobotDiagram(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc.drake.planning;

  auto bind_common_scalar_types = [&m, &doc](auto dummy) {
    using T = decltype(dummy);

    DefineTemplateClassWithDefault<RobotDiagram<T>>(
        m, "RobotDiagram", GetPyParam<T>(), doc.RobotDiagram.doc);
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
