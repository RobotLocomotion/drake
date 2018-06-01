#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/van_der_pol/van_der_pol.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(van_der_pol, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::van_der_pol;

  m.doc() = "Bindings for the van_der_pol example.";

  py::module::import("pydrake.systems.framework");

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  py::class_<VanDerPolOscillator<T>, LeafSystem<T>>(m, "VanDerPolOscillator")
      .def(py::init<>());
}

}  // namespace pydrake
}  // namespace drake
