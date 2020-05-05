#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/copyable_unique_ptr.h"

namespace drake {
namespace pydrake {
namespace {

// A simple struct with a value.
struct MyValue {
  double value{0.};
  explicit MyValue(double value_in) : value(value_in) {}
};

// A simple struct with a bare pointer member.
struct MyContainerRawPtr {
  const MyValue* member{nullptr};
};

struct MyContainerUniquePtr {
  explicit MyContainerUniquePtr(MyValue member_in, MyValue copyable_member_in)
      : member(new MyValue(member_in)),
        copyable_member(new MyValue(copyable_member_in)) {}
  std::unique_ptr<MyValue> member;
  copyable_unique_ptr<MyValue> copyable_member;
};

}  // namespace

PYBIND11_MODULE(wrap_test_util, m) {
  py::class_<MyValue>(m, "MyValue")
      .def(py::init<double>(), py::arg("value"))
      .def_readwrite("value", &MyValue::value, py_reference_internal);

  py::class_<MyContainerRawPtr> my_container(m, "MyContainerRawPtr");
  my_container  // BR
      .def(py::init());
  DefReadWriteKeepAlive(&my_container, "member", &MyContainerRawPtr::member,
      "MyContainerRawPtr doc");

  py::class_<MyContainerUniquePtr> my_unique(m, "MyContainerUniquePtr");
  my_unique.def(py::init<MyValue, MyValue>(), py::arg("member"),
      py::arg("copyable_member"));
  DefReadUniquePtr(&my_unique, "member", &MyContainerUniquePtr::member,
      "MyContainerUniquePtr doc");
  DefReadUniquePtr(&my_unique, "copyable_member",
      &MyContainerUniquePtr::copyable_member, "MyContainerUniquePtr doc");
}

}  // namespace pydrake
}  // namespace drake
