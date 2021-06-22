#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace {

class CustomType {
 public:
  bool operator==(const CustomType&) const { return true; }
};

class MoveOnlyType {
 public:
  explicit MoveOnlyType(int x) : x_(x) {}
  MoveOnlyType(MoveOnlyType&&) = default;
  MoveOnlyType& operator=(MoveOnlyType&&) = default;
  MoveOnlyType(const MoveOnlyType&) = delete;
  MoveOnlyType& operator=(const MoveOnlyType&) = delete;
  int x() const { return x_; }
  void set_x(int x) { x_ = x; }
  std::unique_ptr<MoveOnlyType> Clone() const {
    return std::make_unique<MoveOnlyType>(x_);
  }

 private:
  int x_{};
};

// This type is explicitly not registered with pybind11.
struct UnregisteredType {
  int junk{};
};

}  // namespace

PYBIND11_MODULE(value_test_util, m) {
  py::module::import("pydrake.common");

  py::class_<MoveOnlyType>(m, "MoveOnlyType")
      .def(py::init<int>())
      .def("x", &MoveOnlyType::x)
      .def("set_x", &MoveOnlyType::set_x);
  // Define `Value[MoveOnlyType]` instantiation.
  AddValueInstantiation<MoveOnlyType>(m);

  py::class_<CustomType>(m, "CustomType")
      .def(py::init())
      .def(py::self == py::self);
  // Define `Value[List[CustomType]]` instantiation.
  AddValueInstantiation<std::vector<CustomType>>(m);

  m.def("make_abstract_value_cc_type_unregistered",
      []() { return AbstractValue::Make<UnregisteredType>(); });
}

}  // namespace pydrake
}  // namespace drake
