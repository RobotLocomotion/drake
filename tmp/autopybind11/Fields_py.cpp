#include "drake/perception/point_cloud_flags.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

using namespace drake::perception::pc_flags;

namespace py = pybind11;
void apb11_pydrake_Fields_py_register(py::module &m) {
  py::class_<Fields> PyFields(m, "Fields");

  PyFields.def(py::init<Fields const &>(), py::arg("arg0"))
      .def(py::init<BaseFieldT, DescriptorType>(), py::arg("base_fields"),
           py::arg("descriptor_type"))
      .def(py::init<BaseFieldT>(), py::arg("base_fields"))
      .def(py::init<DescriptorType const &>(), py::arg("descriptor_type"))
      .def("base_fields",
           static_cast<BaseFieldT (Fields::*)() const>(&Fields::base_fields))
      .def("contains",
           static_cast<bool (Fields::*)(Fields const &) const>(
               &Fields::contains),
           py::arg("rhs"))
      .def("descriptor_type",
           static_cast<DescriptorType const &(Fields::*)() const>(
               &Fields::descriptor_type))
      .def("empty", static_cast<bool (Fields::*)() const>(&Fields::empty))
      .def("has_base_fields",
           static_cast<bool (Fields::*)() const>(&Fields::has_base_fields))
      .def("has_descriptor",
           static_cast<bool (Fields::*)() const>(&Fields::has_descriptor))

      .def("__neq__",
           static_cast<bool (Fields::*)(Fields const &) const>(
               &Fields::operator!=),
           py::arg("rhs"))
      .def("__and__",
           static_cast<Fields (Fields::*)(Fields const &) const>(
               &Fields::operator&),
           py::arg("rhs"))
      .def(
          "__iand__",
          static_cast<Fields &(Fields::*)(Fields const &)>(&Fields::operator&=),
          py::arg("rhs"))
      .def(
          "__str__",
          +[](Fields const &rhs) {
            std::ostringstream oss;
            oss << rhs;
            std::string s(oss.str());

            return s;
          })
      .def("__eq__",
           static_cast<bool (Fields::*)(Fields const &) const>(
               &Fields::operator==),
           py::arg("rhs"))
      .def("__or__",
           static_cast<Fields (Fields::*)(Fields const &) const>(
               &Fields::operator|),
           py::arg("rhs"))
      .def(
          "__ior__",
          static_cast<Fields &(Fields::*)(Fields const &)>(&Fields::operator|=),
          py::arg("rhs"));
}
