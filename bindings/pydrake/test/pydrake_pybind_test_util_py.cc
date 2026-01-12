// @file
// Test binding helper methods from `pydrake_pybind_test`.
// @note The main test code that uses these bindings is in
// `pydrake_pybind_test.py`.
#include <memory>
#include <vector>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace pydrake {
namespace {

// Statically check the spelling of the widely-used alias.
static_assert(
    std::is_same_v<py_rvp, py::return_value_policy>, "Alias is wrong?");

struct Item {
  int value{};
};

class ExamplePyKeepAlive {
 public:
  const Item* a() const { return &a_; }
  std::vector<const Item*> a_list() const { return {&a_}; }

 private:
  Item a_{.value = 10};
};

// Class which has a copy constructor, for testing `DefCopyAndDeepCopy`.
struct ExampleDefCopyAndDeepCopy {
  explicit ExampleDefCopyAndDeepCopy(int v) : value(v) {}
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ExampleDefCopyAndDeepCopy);
  int value{};
  bool operator==(const ExampleDefCopyAndDeepCopy& other) const {
    return value == other.value;
  }
};

// Class which has a `Clone()` method and whose copy constructor is explicitly
// disabled, for testing `DefClone`.
class ExampleDefClone {
 public:
  explicit ExampleDefClone(int value) : value_(value) {}
  ExampleDefClone(ExampleDefClone&&) = delete;
  ExampleDefClone& operator=(ExampleDefClone&) = delete;

  std::unique_ptr<ExampleDefClone> Clone() const {
    return std::unique_ptr<ExampleDefClone>(new ExampleDefClone(*this));
  }

  bool operator==(const ExampleDefClone& other) const {
    return value_ == other.value_;
  }

 private:
  ExampleDefClone(const ExampleDefClone&) = default;
  ExampleDefClone& operator=(const ExampleDefClone&) = default;

  int value_{};
};

// Struct which defines attributes which are to be exposed with
// `.def_readwrite`, for testing `ParamInit`.
struct ExampleParamInit {
  int a{0};
  int b{1};
};

}  // namespace

PYBIND11_MODULE(pydrake_pybind_test_util, m) {
  {
    using Class = Item;
    py::class_<Class>(m, "Item").def_readonly("value", &Class::value);
  }
  {
    using Class = ExamplePyKeepAlive;
    py::class_<Class>(m, "ExamplePyKeepAlive")
        .def(py::init())
        .def("a", &Class::a, py_rvp::reference_internal)
        .def("a_list", &Class::a_list, py_rvp::reference_internal);
  }

  {
    using Class = ExampleDefCopyAndDeepCopy;
    py::class_<Class> cls(m, "ExampleDefCopyAndDeepCopy");
    cls  // BR
        .def(py::init([](int value) { return Class(value); }))
        .def(py::self == py::self);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = ExampleDefClone;
    py::class_<Class> cls(m, "ExampleDefClone");
    cls  // BR
        .def(py::init<double>())
        .def(py::self == py::self);
    DefClone(&cls);
  }

  {
    using Class = ExampleParamInit;
    py::class_<Class>(m, "ExampleParamInit")
        .def(ParamInit<Class>())
        .def_readwrite("a", &Class::a)
        .def_readwrite("b", &Class::b)
        // This is purely a sugar method for testing the values.
        .def("compare_values", [](const Class& self, int a, int b) {
          return self.a == a && self.b == b;
        });
  }
}

}  // namespace pydrake
}  // namespace drake
