#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/gen/acrobot_input.h"
#include "drake/examples/acrobot/gen/acrobot_params.h"
#include "drake/examples/acrobot/gen/acrobot_state.h"

using std::make_unique;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(acrobot, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::acrobot;

  m.doc() = "Bindings for the Acrobot example.";

  py::module::import("pydrake.systems.framework");

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion. Issue #7660.
  using T = double;

  py::class_<AcrobotPlant<T>, LeafSystem<T>>(m, "AcrobotPlant")
      .def(py::init<>())
      .def("CalcPotentialEnergy", &AcrobotPlant<T>::CalcPotentialEnergy)
      .def("CalcKineticEnergy", &AcrobotPlant<T>::CalcKineticEnergy)
      .def("DynamicsBiasTerm", &AcrobotPlant<T>::DynamicsBiasTerm)
      .def("MassMatrix", &AcrobotPlant<T>::MassMatrix);

  // TODO(russt): Remove custom bindings once #8096 is resolved.
  py::class_<AcrobotInput<T>, BasicVector<T>>(m, "AcrobotInput")
      .def(py::init<>())
      .def("tau", &AcrobotInput<T>::tau)
      .def("set_tau", &AcrobotInput<T>::set_tau);

  py::class_<AcrobotParams<T>, BasicVector<T>>(m, "AcrobotParams")
      .def(py::init<>())
      .def("m1", &AcrobotParams<T>::m1)
      .def("m2", &AcrobotParams<T>::m2)
      .def("l1", &AcrobotParams<T>::l1)
      .def("lc1", &AcrobotParams<T>::lc1)
      .def("lc2", &AcrobotParams<T>::lc2)
      .def("Ic1", &AcrobotParams<T>::Ic1)
      .def("Ic2", &AcrobotParams<T>::Ic2)
      .def("b1", &AcrobotParams<T>::b1)
      .def("b2", &AcrobotParams<T>::b2)
      .def("gravity", &AcrobotParams<T>::gravity)
      .def("set_m1", &AcrobotParams<T>::set_m1)
      .def("set_m2", &AcrobotParams<T>::set_m2)
      .def("set_l1", &AcrobotParams<T>::set_l1)
      .def("set_lc1", &AcrobotParams<T>::set_lc1)
      .def("set_lc2", &AcrobotParams<T>::set_lc2)
      .def("set_Ic1", &AcrobotParams<T>::set_Ic1)
      .def("set_Ic2", &AcrobotParams<T>::set_Ic2)
      .def("set_b1", &AcrobotParams<T>::set_b1)
      .def("set_b2", &AcrobotParams<T>::set_b2)
      .def("set_gravity", &AcrobotParams<T>::set_gravity);

  py::class_<AcrobotState<T>, BasicVector<T>>(m, "AcrobotState")
      .def(py::init<>())
      .def("theta1", &AcrobotState<T>::theta1)
      .def("theta1dot", &AcrobotState<T>::theta1dot)
      .def("theta2", &AcrobotState<T>::theta2)
      .def("theta2dot", &AcrobotState<T>::theta2dot)
      .def("set_theta1", &AcrobotState<T>::set_theta1)
      .def("set_theta1dot", &AcrobotState<T>::set_theta1dot)
      .def("set_theta2", &AcrobotState<T>::set_theta2)
      .def("set_theta2dot", &AcrobotState<T>::set_theta2dot);
}

}  // namespace pydrake
}  // namespace drake
