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

  explicit TestDummyBase(IsDynamic*) {}
  void AddNot(NotDynamic*) {}
  NotDynamic* ReturnNot() { return new NotDynamic(); }
  NotDynamic* ReturnNullNot() { return nullptr; }
  void AddIs(IsDynamic*) {}
  IsDynamic* ReturnIs() { return new IsDynamic(); }
  IsDynamic* ReturnNullIs() { return nullptr; }
};

}  // namespace

PYBIND11_MODULE(ref_cycle_test_util, m) {
  // The classes refer to each other so we must declare both before defining.
  py::class_<IsDynamic> cls_is_dynamic(m, "IsDynamic", py::dynamic_attr());
  py::class_<NotDynamic> cls_not_dynamic(m, "NotDynamic");

  using internal::ref_cycle;
  {
    using Class = NotDynamic;
    cls_not_dynamic  // BR
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
    cls_is_dynamic  // BR
        .def(py::init<>())
        .def(py::init<IsDynamic*>(), py::arg("thing"), ref_cycle<1, 2>())
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

  m.def("free_function", [](IsDynamic*, IsDynamic*) {}, ref_cycle<1, 2>());
  m.def("invalid_arg_index", [] {}, ref_cycle<0, 1>());
  // Returns its argument and creates a self-cycle.
  m.def("ouroboros", [](IsDynamic* x) { return x; }, ref_cycle<0, 1>());
  m.def("arbitrary_ok", []() {
    auto d1 = py::cast(new IsDynamic);
    auto d2 = py::cast(new IsDynamic);
    internal::make_arbitrary_ref_cycle(d1, d2, "IsDynamic::arbitrary_ok");
    return d1;
  });
  m.def("arbitrary_bad", []() {
    auto dyn = py::cast(new IsDynamic);
    auto bad = py::cast(new NotDynamic);
    internal::make_arbitrary_ref_cycle(dyn, bad, "IsDynamic::arbitrary_bad");
    return dyn;
  });
}

}  // namespace pydrake
}  // namespace drake
