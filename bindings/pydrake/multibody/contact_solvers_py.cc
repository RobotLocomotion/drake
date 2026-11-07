#include "drake/bindings/generated_docstrings/multibody_cenic.h"
#include "drake/bindings/generated_docstrings/multibody_contact_solvers_icf.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/cenic/cenic_integrator_parameters.h"
#include "drake/multibody/contact_solvers/icf/icf_solver_parameters.h"

namespace drake {
namespace pydrake {
namespace {

void DefineIcf(py::module_ m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::contact_solvers::icf;
  constexpr auto& doc = pydrake_doc_multibody_contact_solvers_icf.drake
                            .multibody.contact_solvers.icf;

  // IcfSolverParameters
  {
    using Class = IcfSolverParameters;
    constexpr auto& cls_doc = doc.IcfSolverParameters;
    class_<Class> cls(
        m, "IcfSolverParameters", py::dynamic_attr(), cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }
}

// N.B. CenicIntegratorParameters is bound here (rather than in the `cenic`
// module) so that `pydrake.systems.analysis` can use it as part of
// SimulatorConfig without importing the `cenic` module, which itself imports
// `pydrake.systems.analysis`.
void DefineCenicIntegratorParameters(py::module_ m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc_multibody_cenic.drake.multibody;

  using Class = CenicIntegratorParameters;
  constexpr auto& cls_doc = doc.CenicIntegratorParameters;
  class_<Class> cls(
      m, "CenicIntegratorParameters", py::dynamic_attr(), cls_doc.doc);
  cls  // BR
      .def(ParamInit<Class>());
  DefAttributesUsingSerialize(&cls, cls_doc);
  DefReprUsingSerialize(&cls);
  DefCopyAndDeepCopy(&cls);
}

PYDRAKE_MODULE(contact_solvers, m) {
  DefineIcf(m);
  DefineCenicIntegratorParameters(m);
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
