#include "drake/math/autodiff.h"
#include "external/eigen/include/_usr_include_eigen3/Eigen/src/Core/Matrix.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

using namespace drake::math;

namespace py = pybind11;
void apb11_pydrake_AutoDiffXd_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  py::class_<drake::AutoDiffXd> PyAutoDiffXd(m, "AutoDiffXd");

  PyAutoDiffXd.def(py::init<>())
    .def(py::init<Eigen::Ref<drake::AutoDiffXd::Scalar const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &,int,int>(),py::arg("value"),py::arg("nbDer"),py::arg("derNumber"))
    .def(py::init<Eigen::Ref<drake::AutoDiffXd::Real const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),py::arg("value"))
    .def(py::init<Eigen::Ref<drake::AutoDiffXd::Scalar const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &,Eigen::Ref<drake::AutoDiffXd::DerType const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),py::arg("value"),py::arg("der"))
    .def(py::init<Eigen::Ref<drake::AutoDiffXd const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &,::Eigen::internal::enable_if<true, void *>::type>(),py::arg("other"),py::arg("arg1") = (::Eigen::internal::enable_if<true, void *>::type)0)
    .def(py::init<Eigen::Ref<drake::AutoDiffXd const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),py::arg("other"))
    .def("derivatives", static_cast<drake::AutoDiffXd::DerType const & ( drake::AutoDiffXd::* )(  )const>(&drake::AutoDiffXd::derivatives), py::return_value_policy::reference_internal)
    .def("derivatives", static_cast<drake::AutoDiffXd::DerType & ( drake::AutoDiffXd::* )(  )>(&drake::AutoDiffXd::derivatives), py::return_value_policy::reference_internal)
    .def("value", static_cast<drake::AutoDiffXd::Scalar const & ( drake::AutoDiffXd::* )(  )const>(&drake::AutoDiffXd::value), py::return_value_policy::reference_internal)
    .def("value", static_cast<drake::AutoDiffXd::Scalar & ( drake::AutoDiffXd::* )(  )>(&drake::AutoDiffXd::value), py::return_value_policy::reference_internal)
    
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
