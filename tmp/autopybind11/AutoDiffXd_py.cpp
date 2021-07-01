#include "drake/math/autodiff.h"
#include "external/eigen/include/_usr_include_eigen3/Eigen/src/Core/Matrix.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

using namespace drake::math;

namespace py = pybind11;

template <typename Class, typename... Options>
py::class_<Class, Options...> DefineTemplateClass(py::handle scope,
                                                  const char *name,
                                                  const char *doc_string = "") {
  py::class_<Class, Options...> py_class(scope, name, doc_string);
  return py_class;
};

void apb11_pydrake_AutoDiffXd_py_register(py::module &m) {
  // Instantiation of AutoDiffScalar<Eigen::VectorXd>
  auto PyAutoDiffXd = DefineTemplateClass<drake::AutoDiffXd>(m, "AutoDiffXd");

  PyAutoDiffXd.def(py::init<>())
    .def(py::init<drake::AutoDiffXd::Scalar const &,int,int>(),py::arg("value"),py::arg("nbDer"),py::arg("derNumber"))
    .def(py::init<drake::AutoDiffXd::Real const &>(),py::arg("value"))
    .def(py::init<drake::AutoDiffXd::Scalar const &,drake::AutoDiffXd::DerType const &>(),py::arg("value"),py::arg("der"))
    .def(py::init<drake::AutoDiffXd const &,::Eigen::internal::enable_if<true, void *>::type>(),py::arg("other"),py::arg("arg1") = (::Eigen::internal::enable_if<true, void *>::type)0)
    .def(py::init<drake::AutoDiffXd const &>(),py::arg("other"))
    .def("derivatives", static_cast<drake::AutoDiffXd::DerType const & ( drake::AutoDiffXd::* )(  )const>(&drake::AutoDiffXd::derivatives))
    .def("derivatives", static_cast<drake::AutoDiffXd::DerType & ( drake::AutoDiffXd::* )(  )>(&drake::AutoDiffXd::derivatives))
    .def("value", static_cast<drake::AutoDiffXd::Scalar const & ( drake::AutoDiffXd::* )(  )const>(&drake::AutoDiffXd::value))
    .def("value", static_cast<drake::AutoDiffXd::Scalar & ( drake::AutoDiffXd::* )(  )>(&drake::AutoDiffXd::value))
    
    .def("__neq__", static_cast<bool ( drake::AutoDiffXd::* )( drake::AutoDiffXd::Scalar const & )const>(&drake::AutoDiffXd::operator!=), py::arg("other"))
    .def("__neq__", +[](drake::AutoDiffXd::Scalar const & a,drake::AutoDiffXd const & b){ return a!=b; })
    .def("__neq__", static_cast<bool ( drake::AutoDiffXd::* )( drake::AutoDiffXd const & )const>(&drake::AutoDiffXd::operator!=), py::arg("b"))
    .def("__mul__", +[](drake::AutoDiffXd a,drake::AutoDiffXd::Scalar const & b){ return a*b; })
    .def("__mul__", +[](drake::AutoDiffXd::Scalar const & a,drake::AutoDiffXd b){ return a*b; })
    .def("__imul__", static_cast<drake::AutoDiffXd & ( drake::AutoDiffXd::* )( drake::AutoDiffXd::Scalar const & )>(&drake::AutoDiffXd::operator*=), py::arg("other"))
    .def("__imul__", static_cast<drake::AutoDiffXd & ( drake::AutoDiffXd::* )( drake::AutoDiffXd const & )>(&drake::AutoDiffXd::operator*=), py::arg("other"))
    .def("__add__", +[](drake::AutoDiffXd a,drake::AutoDiffXd::Scalar const & b){ return a+b; })
    .def("__add__", +[](drake::AutoDiffXd::Scalar const & a,drake::AutoDiffXd b){ return a+b; })
    .def("__iadd__", static_cast<drake::AutoDiffXd & ( drake::AutoDiffXd::* )( drake::AutoDiffXd::Scalar const & )>(&drake::AutoDiffXd::operator+=), py::arg("other"))
    .def("__iadd__", static_cast<drake::AutoDiffXd & ( drake::AutoDiffXd::* )( drake::AutoDiffXd const & )>(&drake::AutoDiffXd::operator+=), py::arg("other"))
    .def("__sub__", +[](drake::AutoDiffXd a,drake::AutoDiffXd::Scalar const & b){ return a-b; })
    .def("__sub__", +[](drake::AutoDiffXd::Scalar const & a,drake::AutoDiffXd b){ return a-b; })
    .def("__sub__", +[](drake::AutoDiffXd a){ return a; })
    .def("__isub__", static_cast<drake::AutoDiffXd & ( drake::AutoDiffXd::* )( drake::AutoDiffXd::Scalar const & )>(&drake::AutoDiffXd::operator-=), py::arg("other"))
    .def("__isub__", static_cast<drake::AutoDiffXd & ( drake::AutoDiffXd::* )( drake::AutoDiffXd const & )>(&drake::AutoDiffXd::operator-=), py::arg("other"))
    .def("__truediv__", +[](drake::AutoDiffXd a,drake::AutoDiffXd::Scalar const & b){ return a/b; })
    .def("__truediv__", +[](drake::AutoDiffXd::Scalar const & a,drake::AutoDiffXd b){ return a/b; })
    .def("__itruediv__", static_cast<drake::AutoDiffXd & ( drake::AutoDiffXd::* )( drake::AutoDiffXd::Scalar const & )>(&drake::AutoDiffXd::operator/=), py::arg("other"))
    .def("__itruediv__", static_cast<drake::AutoDiffXd & ( drake::AutoDiffXd::* )( drake::AutoDiffXd const & )>(&drake::AutoDiffXd::operator/=), py::arg("other"))
    .def("__lt__", static_cast<bool ( drake::AutoDiffXd::* )( drake::AutoDiffXd::Scalar const & )const>(&drake::AutoDiffXd::operator<), py::arg("other"))
    .def("__lt__", +[](drake::AutoDiffXd::Scalar const & a,drake::AutoDiffXd const & b){ return a<b; })
    .def("__lt__", static_cast<bool ( drake::AutoDiffXd::* )( drake::AutoDiffXd const & )const>(&drake::AutoDiffXd::operator<), py::arg("b"))
    .def("__str__", +[](drake::AutoDiffXd const & a) {
        std::ostringstream oss;
        oss << a;
        std::string s(oss.str());

        return s;})    .def("__le__", static_cast<bool ( drake::AutoDiffXd::* )( drake::AutoDiffXd::Scalar const & )const>(&drake::AutoDiffXd::operator<=), py::arg("other"))
    .def("__le__", +[](drake::AutoDiffXd::Scalar const & a,drake::AutoDiffXd const & b){ return a<=b; })
                .def("__eq__", static_cast<bool ( drake::AutoDiffXd::* )( drake::AutoDiffXd::Scalar const & )const>(&drake::AutoDiffXd::operator==), py::arg("other"))
    .def("__eq__", +[](drake::AutoDiffXd::Scalar const & a,drake::AutoDiffXd const & b){ return a==b; })
    .def("__eq__", static_cast<bool ( drake::AutoDiffXd::* )( drake::AutoDiffXd const & )const>(&drake::AutoDiffXd::operator==), py::arg("b"))
    .def("__gt__", static_cast<bool ( drake::AutoDiffXd::* )( drake::AutoDiffXd::Scalar const & )const>(&drake::AutoDiffXd::operator>), py::arg("other"))
    .def("__gt__", +[](drake::AutoDiffXd::Scalar const & a,drake::AutoDiffXd const & b){ return a>b; })
    .def("__gt__", static_cast<bool ( drake::AutoDiffXd::* )( drake::AutoDiffXd const & )const>(&drake::AutoDiffXd::operator>), py::arg("b"))
    .def("__ge__", static_cast<bool ( drake::AutoDiffXd::* )( drake::AutoDiffXd::Scalar const & )const>(&drake::AutoDiffXd::operator>=), py::arg("other"))
    .def("__ge__", +[](drake::AutoDiffXd::Scalar const & a,drake::AutoDiffXd const & b){ return a>=b; })
    .def("__ge__", static_cast<bool ( drake::AutoDiffXd::* )( drake::AutoDiffXd const & )const>(&drake::AutoDiffXd::operator>=), py::arg("b"))
    ;
}
