#include "drake/math/autodiff.h"
#include "external/eigen/include/_usr_include_eigen3/Eigen/src/Core/Matrix.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

namespace py = pybind11;
void apb11_pydrake_AutoDiffXd_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::math;

  using PyAutoDiffXd_0 = Eigen::VectorXd;

  py::class_<::Eigen::AutoDiffScalar<PyAutoDiffXd_0>> PyAutoDiffXd(
      m, "AutoDiffXd");

  PyAutoDiffXd.def(py::init<>())
    .def(py::init<Eigen::Ref<::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &,int,int>(),py::arg("value"),py::arg("nbDer"),py::arg("derNumber"))
    .def(py::init<Eigen::Ref<::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Real const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),py::arg("value"))
    .def(py::init<Eigen::Ref<::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &,Eigen::Ref<::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::DerType const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),py::arg("value"),py::arg("der"))
    .def(py::init<Eigen::Ref<::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &,::Eigen::internal::enable_if<true, void *>::type>(),py::arg("other"),py::arg("arg1") = (::Eigen::internal::enable_if<true, void *>::type)0)
    .def(py::init<Eigen::Ref<::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const &, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),py::arg("other"))
    .def("derivatives", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::DerType const & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )(  )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::derivatives), py::return_value_policy::reference_internal)
    .def("derivatives", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::DerType & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )(  )>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::derivatives), py::return_value_policy::reference_internal)
    .def("value", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )(  )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::value), py::return_value_policy::reference_internal)
    .def("value", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )(  )>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::value), py::return_value_policy::reference_internal)
    
    .def("__neq__", static_cast<bool ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator!=), py::arg("other"))
    .def("__neq__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & b){ return a!=b; })
    .def("__neq__", static_cast<bool ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator!=), py::arg("b"))
    .def("__mul__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0> a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & b){ return a*b; })
    .def("__mul__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0> b){ return a*b; })
    .def("__imul__", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0> & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & )>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator*=), py::arg("other"))
    .def("__imul__", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0> & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & )>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator*=), py::arg("other"))
    .def("__add__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0> a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & b){ return a+b; })
    .def("__add__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0> b){ return a+b; })
    .def("__iadd__", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0> & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & )>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator+=), py::arg("other"))
    .def("__iadd__", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0> & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & )>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator+=), py::arg("other"))
    .def("__sub__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0> a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & b){ return a-b; })
    .def("__sub__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0> b){ return a-b; })
    .def("__sub__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0> a){ return a; })
    .def("__isub__", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0> & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & )>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator-=), py::arg("other"))
    .def("__isub__", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0> & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & )>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator-=), py::arg("other"))
    .def("__truediv__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0> a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & b){ return a/b; })
    .def("__truediv__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0> b){ return a/b; })
    .def("__itruediv__", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0> & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & )>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator/=), py::arg("other"))
    .def("__itruediv__", static_cast<::Eigen::AutoDiffScalar<PyAutoDiffXd_0> & ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & )>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator/=), py::arg("other"))
    .def("__lt__", static_cast<bool ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator<), py::arg("other"))
    .def("__lt__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & b){ return a<b; })
    .def("__lt__", static_cast<bool ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator<), py::arg("b"))
    .def("__str__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & a) {
        std::ostringstream oss;
        oss << a;
        std::string s(oss.str());

        return s;})    .def("__le__", static_cast<bool ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator<=), py::arg("other"))
    .def("__le__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & b){ return a<=b; })
                .def("__eq__", static_cast<bool ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator==), py::arg("other"))
    .def("__eq__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & b){ return a==b; })
    .def("__eq__", static_cast<bool ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator==), py::arg("b"))
    .def("__gt__", static_cast<bool ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator>), py::arg("other"))
    .def("__gt__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & b){ return a>b; })
    .def("__gt__", static_cast<bool ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator>), py::arg("b"))
    .def("__ge__", static_cast<bool ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator>=), py::arg("other"))
    .def("__ge__", +[](::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::Scalar const & a,::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & b){ return a>=b; })
    .def("__ge__", static_cast<bool ( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::* )( ::Eigen::AutoDiffScalar<PyAutoDiffXd_0> const & )const>(&::Eigen::AutoDiffScalar<PyAutoDiffXd_0>::operator>=), py::arg("b"))
    ;
}
