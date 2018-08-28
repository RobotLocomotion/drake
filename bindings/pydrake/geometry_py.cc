#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace pydrake {

using systems::LeafSystem;

// TODO(eric.cousineau): Bind additional scalar types.
using T = double;

PYBIND11_MODULE(geometry, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;

  py::module::import("pydrake.systems.framework");
  py::class_<SceneGraph<T>, LeafSystem<T>>(m, "SceneGraph");
}

}  // namespace pydrake
}  // namespace drake
