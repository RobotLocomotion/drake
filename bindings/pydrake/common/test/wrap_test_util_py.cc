#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace {

// A simple struct with a value.
struct MyValue {
  double value{0.};
};

// A simple struct with a bare pointer member.
struct MyContainer {
  const MyValue* member{nullptr};
};

}  // namespace

PYBIND11_MODULE(wrap_test_util, m) {
  py::class_<MyValue>(m, "MyValue")
      .def(py::init<>())
      .def_readwrite("value", &MyValue::value, py_reference_internal);

  py::class_<MyContainer> my_container(m, "MyContainer");
  my_container  // BR
      .def(py::init<>());
  DefReadWriteKeepAlive(&my_container, "member", &MyContainer::member);
}

}  // namespace pydrake
}  // namespace drake
