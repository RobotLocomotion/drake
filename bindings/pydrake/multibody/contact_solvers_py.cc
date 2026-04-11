#include "drake/bindings/generated_docstrings/multibody_contact_solvers_icf.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/contact_solvers/icf/icf_solver_parameters.h"

namespace drake {
namespace pydrake {
namespace {

void DefineIcf(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::contact_solvers::icf;
  constexpr auto& doc = pydrake_doc_multibody_contact_solvers_icf.drake
                            .multibody.contact_solvers.icf;

  // IcfSolverParameters
  {
    using Class = IcfSolverParameters;
    constexpr auto& cls_doc = doc.IcfSolverParameters;
    py::class_<Class> cls(
        m, "IcfSolverParameters", py::dynamic_attr(), cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }
}

PYBIND11_MODULE(contact_solvers, m) {
  DefineIcf(m);
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
