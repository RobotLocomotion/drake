// Bindings that help test the ref_cycle<>() annotation.
// See also ref_cycle_test.py.

#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

namespace {

template <bool AmIDynamic>
class TestDummyBase;

using IsDynamic = TestDummyBase<true>;
using NotDynamic = TestDummyBase<false>;

template <bool AmIDynamic>
class TestDummyBase {
 public:
  TestDummyBase() = default;
  TestDummyBase(const TestDummyBase&) = default;
  ~TestDummyBase() = default;

  void AddNot(NotDynamic*) {}
  NotDynamic* ReturnNot() { return new NotDynamic(); }
  NotDynamic* ReturnNullNot() { return nullptr; }
  void AddIs(IsDynamic*) {}
  IsDynamic* ReturnIs() { return new IsDynamic(); }
  IsDynamic* ReturnNullIs() { return nullptr; }
};

}  // namespace

PYBIND11_MODULE(ref_cycle_test_util, m) {
  using internal::ref_cycle;
  {
    using Class = NotDynamic;
    py::class_<Class>(m, "NotDynamic")
        .def(py::init<>())
        .def("AddIs", &Class::AddIs)
        .def("AddIsCycle", &Class::AddIs, ref_cycle<1, 2>())
        .def("ReturnIs", &Class::ReturnIs)
        .def("ReturnIsCycle", &Class::ReturnIs, ref_cycle<0, 1>())
        .def("ReturnNullIs", &Class::ReturnNullIs)
        .def("ReturnNullIsCycle", &Class::ReturnNullIs, ref_cycle<0, 1>());
  }

  {
    using Class = IsDynamic;
    py::class_<Class>(m, "IsDynamic", py::dynamic_attr())
        .def(py::init<>())
        .def("AddNot", &Class::AddNot)
        .def("AddNotCycle", &Class::AddNot, ref_cycle<1, 2>())
        .def("ReturnNot", &Class::ReturnNot)
        .def("ReturnNotCycle", &Class::ReturnNot, ref_cycle<0, 1>())
        .def("ReturnNullNot", &Class::ReturnNullNot)
        .def("ReturnNullNotCycle", &Class::ReturnNullNot, ref_cycle<0, 1>())
        .def("AddIs", &Class::AddIs)
        .def("AddIsCycle", &Class::AddIs, ref_cycle<1, 2>())
        .def("ReturnIs", &Class::ReturnIs)
        .def("ReturnIsCycle", &Class::ReturnIs, ref_cycle<0, 1>())
        .def("ReturnNullIs", &Class::ReturnNullIs)
        .def("ReturnNullIsCycle", &Class::ReturnNullIs, ref_cycle<0, 1>());
  }

  m.def(
      "free_function", [](IsDynamic*, IsDynamic*) {}, ref_cycle<1, 2>());
  m.def(
      "invalid_arg_index", [] {}, ref_cycle<0, 1>());
  // Returns its argument and creates a self-cycle.
  m.def(
      "ouroboros", [](IsDynamic* x) { return x; }, ref_cycle<0, 1>());
}

}  // namespace pydrake
}  // namespace drake
