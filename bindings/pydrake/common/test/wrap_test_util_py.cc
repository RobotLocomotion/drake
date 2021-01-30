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

struct TypeConversionExample {
  std::string value;
};

// Wrapper for TypeConversionExample.
struct wrapper_type_conversion_exaple {
  using Type = TypeConversionExample;
  static constexpr auto original_name = py::detail::_("TypeConversionExample");
  using WrappedType = std::string;
  static constexpr auto wrapped_name = py::detail::_("str");

  static TypeConversionExample unwrap(const std::string& value) {
    return TypeConversionExample{value};
  }
  static std::string wrap(const TypeConversionExample& obj) {
    return obj.value;
  }
};

TypeConversionExample MakeTypeConversionExample() {
  return TypeConversionExample{"hello"};
}

bool CheckTypeConversionExample(const TypeConversionExample& obj) {
  return obj.value == "hello";
}

}  // namespace
}  // namespace pydrake
}  // namespace drake

namespace pybind11 {
namespace detail {
template <>
struct type_caster<drake::pydrake::TypeConversionExample>
    : public drake::pydrake::internal::type_caster_wrapped<
          drake::pydrake::wrapper_type_conversion_exaple> {};
}  // namespace detail
}  // namespace pybind11

namespace drake {
namespace pydrake {
PYBIND11_MODULE(wrap_test_util, m) {
  py::class_<MyValue>(m, "MyValue")
      .def(py::init<double>(), py::arg("value"))
      .def_readwrite("value", &MyValue::value, py_rvp::reference_internal);

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

  m.def("MakeTypeConversionExample", &MakeTypeConversionExample);
  m.def("MakeTypeConversionExampleBadRvp", &MakeTypeConversionExample,
      py_rvp::reference);
  m.def("CheckTypeConversionExample", &CheckTypeConversionExample,
      py::arg("obj"));
}
}  // namespace pydrake
}  // namespace drake
