#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/default_scalars.h"
#include "drake/multibody/fem/deformable_body_config.h"

namespace drake {
namespace pydrake {
namespace {

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::fem;
  constexpr auto& doc = pydrake_doc.drake.multibody.fem;

  {
    using Class = MaterialModel;
    constexpr auto& cls_doc = doc.MaterialModel;
    py::enum_<Class>(m, "MaterialModel", cls_doc.doc)
        .value("kLinearCorotated", Class::kLinearCorotated,
            cls_doc.kLinearCorotated.doc)
        .value("kCorotated", Class::kCorotated, cls_doc.kCorotated.doc)
        .value("kLinear", Class::kLinear, cls_doc.kLinear.doc);
  }
}

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::fem;
  constexpr auto& doc = pydrake_doc.drake.multibody.fem;

  {
    using Class = DeformableBodyConfig<T>;
    constexpr auto& cls_doc = doc.DeformableBodyConfig;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "DeformableBodyConfig", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("set_youngs_modulus", &Class::set_youngs_modulus,
            py::arg("youngs_modulus"), cls_doc.set_youngs_modulus.doc)
        .def("set_poissons_ratio", &Class::set_poissons_ratio,
            py::arg("poissons_ratio"), cls_doc.set_poissons_ratio.doc)
        .def("set_mass_damping_coefficient",
            &Class::set_mass_damping_coefficient,
            py::arg("mass_damping_coefficient"),
            cls_doc.set_mass_damping_coefficient.doc)
        .def("set_stiffness_damping_coefficient",
            &Class::set_stiffness_damping_coefficient,
            py::arg("stiffness_damping_coefficient"),
            cls_doc.set_stiffness_damping_coefficient.doc)
        .def("set_mass_density", &Class::set_mass_density,
            py::arg("mass_density"), cls_doc.set_mass_density.doc)
        .def("set_material_model", &Class::set_material_model,
            py::arg("material_model"), cls_doc.set_material_model.doc)
        .def("youngs_modulus", &Class::youngs_modulus,
            py_rvp::reference_internal, cls_doc.youngs_modulus.doc)
        .def("poissons_ratio", &Class::poissons_ratio,
            py_rvp::reference_internal, cls_doc.poissons_ratio.doc)
        .def("mass_damping_coefficient", &Class::mass_damping_coefficient,
            py_rvp::reference_internal, cls_doc.mass_damping_coefficient.doc)
        .def("stiffness_damping_coefficient",
            &Class::stiffness_damping_coefficient, py_rvp::reference_internal,
            cls_doc.stiffness_damping_coefficient.doc)
        .def("mass_density", &Class::mass_density, py_rvp::reference_internal,
            cls_doc.mass_density.doc)
        .def("material_model", &Class::material_model,
            cls_doc.material_model.doc);
    DefCopyAndDeepCopy(&cls);
  }
}
}  // namespace

PYBIND11_MODULE(fem, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  m.doc() = "Bindings for multibody fem.";

  py::module::import("pydrake.autodiffutils");

  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
