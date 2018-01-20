#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/pendulum/pendulum_plant.h"

using std::make_unique;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(pendulum, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::pendulum;

  m.doc() = "Bindings for the Pendulum example.";

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  py::class_<PendulumPlant<T>, LeafSystem<T>>(m, "PendulumPlant")
    .def(py::init<>());
}

}  // namespace pydrake
}  // namespace drake
