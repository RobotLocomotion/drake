#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/monostate_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/meshcat/joint_sliders.h"

using drake::multibody::MultibodyPlant;
using drake::systems::LeafSystem;

namespace drake {
namespace pydrake {

namespace {
template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::meshcat;
  constexpr auto& doc = pydrake_doc.drake.multibody.meshcat;

  // JointSliders
  {
    using Class = JointSliders<T>;
    constexpr auto& cls_doc = doc.JointSliders;
    DefineTemplateClassWithDefault<JointSliders<T>, LeafSystem<T>>(
        m, "JointSliders", param, doc.JointSliders.doc)
        .def(py::init<std::shared_ptr<geometry::Meshcat>,
                 const MultibodyPlant<T>*, std::optional<Eigen::VectorXd>,
                 std::variant<std::monostate, double, Eigen::VectorXd>,
                 std::variant<std::monostate, double, Eigen::VectorXd>,
                 std::variant<std::monostate, double, Eigen::VectorXd>>(),
            py::arg("meshcat"), py::arg("plant"),
            py::arg("initial_value") = py::none(),
            py::arg("lower_limit") = py::none(),
            py::arg("upper_limit") = py::none(), py::arg("step") = py::none(),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 3>(),  // BR
            cls_doc.ctor.doc)
        .def("Delete", &Class::Delete, cls_doc.Delete.doc)
        .def("Run", &Class::Run, py::arg("diagram"),
            py::arg("timeout") = py::none(), cls_doc.Run.doc);
  }
}
}  // namespace

PYBIND11_MODULE(meshcat, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  m.doc() = "Interface code for Meshcat-based visualization";

  py::module::import("pydrake.multibody.plant");

  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
