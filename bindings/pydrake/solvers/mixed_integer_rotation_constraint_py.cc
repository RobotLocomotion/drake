#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/solvers/mixed_integer_optimization_util.h"
#include "drake/solvers/mixed_integer_rotation_constraint.h"

namespace drake {
namespace pydrake {
PYBIND11_MODULE(mixed_integer_rotation_constraint, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "mixed integer rotation constraint bindings";

  py::module::import("pydrake.solvers.mathematicalprogram");

  {
    using Class = MixedIntegerRotationConstraintGenerator;
    constexpr auto& cls_doc = doc.MixedIntegerRotationConstraintGenerator;
    py::class_<Class> cls(
        m, "MixedIntegerRotationConstraintGenerator", cls_doc.doc);

    using Enum = Class::Approach;
    constexpr auto& enum_doc = cls_doc.Approach;
    py::enum_<Class::Approach>(cls, "Approach", enum_doc.doc)
        .value("kBoxSphereIntersection", Enum::kBoxSphereIntersection,
            enum_doc.kBoxSphereIntersection.doc)
        .value("kBilinearMcCormick", Enum::kBilinearMcCormick,
            enum_doc.kBilinearMcCormick.doc)
        .value("kBoth", Enum::kBoth, enum_doc.kBoth.doc);

    using Struct = Class::ReturnType;
    constexpr auto& struct_doc = cls_doc.ReturnType;
    py::class_<Struct>(cls, "ReturnType", struct_doc.doc);

    cls.def(py::init<Class::Approach, int, IntervalBinning>(),
           py::arg("approach"), py::arg("num_intervals_per_half_axis"),
           py::arg("interval_binning"), cls_doc.ctor.doc)
        .def("AddToProgram", &Class::AddToProgram, py::arg("R"),
            py::arg("prog"), cls_doc.AddToProgram.doc);
  }
}
}  // namespace pydrake
}  // namespace drake
