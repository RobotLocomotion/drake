#include "drake/common/symbolic.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

using namespace drake::symbolic;

namespace py = pybind11;

void apb11_pydrake_Environment_py_register(py::module &m) {
  py::class_<Environment> PyEnvironment(m, "Environment");

  PyEnvironment.def(py::init<Environment const &>(), py::arg("arg0"))
      .def(py::init<>())
      .def(py::init<
               ::std::initializer_list<std::pair<const Variable, double>>>(),
           py::arg("init"))
      .def(py::init<::std::initializer_list<Variable>>(), py::arg("vars"))
      .def(py::init<Environment::map>(), py::arg("m"))
      .def("begin", static_cast<Environment::iterator (Environment::*)()>(
                        &Environment::begin))
      .def("begin",
           static_cast<Environment::const_iterator (Environment::*)() const>(
               &Environment::begin))
      .def("cbegin",
           static_cast<Environment::const_iterator (Environment::*)() const>(
               &Environment::cbegin))
      .def("cend",
           static_cast<Environment::const_iterator (Environment::*)() const>(
               &Environment::cend))
      .def("domain", static_cast<Variables (Environment::*)() const>(
                         &Environment::domain))
      .def("empty",
           static_cast<bool (Environment::*)() const>(&Environment::empty))
      .def("end", static_cast<Environment::iterator (Environment::*)()>(
                      &Environment::end))
      .def("end",
           static_cast<Environment::const_iterator (Environment::*)() const>(
               &Environment::end))
      .def("find",
           static_cast<Environment::iterator (Environment::*)(
               Environment::key_type const &)>(&Environment::find),
           py::arg("key"))
      .def("find",
           static_cast<Environment::const_iterator (Environment::*)(
               Environment::key_type const &) const>(&Environment::find),
           py::arg("key"))
      .def("insert",
           static_cast<void (Environment::*)(Environment::key_type const &,
                                             Environment::mapped_type const &)>(
               &Environment::insert),
           py::arg("key"), py::arg("elem"))
      .def("insert",
           static_cast<void (Environment::*)(
               ::Eigen::Ref<const Eigen::Matrix<Variable, -1, -1, 0, -1, -1>, 0,
                            Eigen::OuterStride<-1>> const &,
               ::Eigen::Ref<const Eigen::Matrix<double, -1, -1, 0, -1, -1>, 0,
                            Eigen::OuterStride<-1>> const &)>(
               &Environment::insert),
           py::arg("keys"), py::arg("elements"))
      .def("size",
           static_cast<drake::symbolic::Variable::Id (Environment::*)() const>(
               &Environment::size))
      .def("to_string", static_cast<::std::string (Environment::*)() const>(
                            &Environment::to_string))

      .def(
          "__str__",
          +[](Environment const &env) {
            std::ostringstream oss;
            oss << env;
            std::string s(oss.str());

            return s;
          })
      .def("__getitem__",
           static_cast<Environment::mapped_type &(
               Environment::*)(Environment::key_type const &)>(
               &Environment::operator[]),
           py::arg("key"))
      .def("__getitem__",
           static_cast<Environment::mapped_type const &(
               Environment::*)(Environment::key_type const &)const>(
               &Environment::operator[]),
           py::arg("key"));
}
