#include <string>

#include "drake/bindings/generated_docstrings/planning.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/dof_mask.h"

namespace drake {
namespace pydrake {
namespace internal {

using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;

void DefinePlanningDofMask(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc_planning.drake.planning;

  {
    using Class = DofMask;
    constexpr auto& cls_doc = doc.DofMask;
    py::class_<Class>(m, "DofMask", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_default)
        .def(py::init<int, bool>(), py::arg("size"), py::arg("value"),
            cls_doc.ctor.doc_by_size)
        .def(py::init<std::vector<bool>>(), py::arg("values"),
            cls_doc.ctor.doc_vector_bool)
        .def_static("MakeFromModel",
            overload_cast_explicit<DofMask, const MultibodyPlant<double>&,
                ModelInstanceIndex>(&Class::MakeFromModel),
            py::arg("plant"), py::arg("model_index"),
            cls_doc.MakeFromModel.doc_2args_plant_model_index)
        .def_static("MakeFromModel",
            overload_cast_explicit<DofMask, const MultibodyPlant<double>&,
                const std::string&>(&Class::MakeFromModel),
            py::arg("plant"), py::arg("model_name"),
            cls_doc.MakeFromModel.doc_2args_plant_model_name)
        .def("size", &Class::size, cls_doc.size.doc)
        .def("count", &Class::count, cls_doc.count.doc)
        .def("GetJoints", &Class::GetJoints, py::arg("plant"),
            cls_doc.GetJoints.doc)
        .def(py::self == py::self)
        .def("__getitem__", &DofMask::operator[], cls_doc.operator_array.doc)
        .def("Complement", &Class::Complement, cls_doc.Complement.doc)
        .def("Union", &Class::Union, py::arg("other"), cls_doc.Union.doc)
        .def("Intersect", &Class::Intersect, py::arg("other"),
            cls_doc.Intersect.doc)
        .def("Subtract", &Class::Subtract, py::arg("other"),
            cls_doc.Subtract.doc)
        .def("GetFullToSelectedIndex", &Class::GetFullToSelectedIndex,
            cls_doc.GetFullToSelectedIndex.doc)
        .def("GetSelectedToFullIndex", &Class::GetSelectedToFullIndex,
            cls_doc.GetSelectedToFullIndex.doc);
    py::implicitly_convertible<std::vector<bool>, Class>();
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
