#include "drake/bindings/generated_docstrings/planning_experimental.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/experimental/placeholder.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningPlaceholder(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning::experimental;
  constexpr auto& doc =
      pydrake_doc_planning_experimental.drake.planning.experimental;

  {
    using Class = Placeholder;
    constexpr auto& cls_doc = doc.Placeholder;
    py::class_<Class>(m, "Placeholder", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
