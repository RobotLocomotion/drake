#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/manipulation/util/zero_force_driver.h"
#include "drake/manipulation/util/zero_force_driver_functions.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineManipulationUtil(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::manipulation;
  constexpr auto& doc = pydrake_doc.drake.manipulation;

  {
    using Class = ZeroForceDriver;
    constexpr auto& cls_doc = doc.ZeroForceDriver;
    py::class_<Class> cls(m, "ZeroForceDriver", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    m.def("ApplyDriverConfig", &ApplyDriverConfig, py::arg("driver_config"),
        py::arg("model_instance_name"), py::arg("sim_plant"),
        py::arg("models_from_directives"), py::arg("lcms"), py::arg("builder"),
        doc.ApplyDriverConfig.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
