#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace pydrake {

using py::handle;
using py::detail::function_call;

namespace internal {

namespace {

void make_ref_cycle(handle p0, handle p1) {
  DRAKE_DEMAND(static_cast<bool>(p0));
  DRAKE_DEMAND(static_cast<bool>(p1));
  DRAKE_DEMAND(!p0.is_none());
  DRAKE_DEMAND(!p1.is_none());
  DRAKE_DEMAND(PyType_IS_GC(Py_TYPE(p0.ptr())));
  DRAKE_DEMAND(PyType_IS_GC(Py_TYPE(p1.ptr())));

  // Each peer will have a new/updated attribute, containing a set of
  // handles. Insert each into the other's handle set. Create the set first
  // if it is not yet existing.
  auto make_link = [](handle a, handle b) {
    static const char refcycle_peers[] = "_pydrake_internal_ref_cycle_peers";
    if (!hasattr(a, refcycle_peers)) {
      py::set new_set;
      DRAKE_DEMAND(PyType_IS_GC(Py_TYPE(new_set.ptr())));
      a.attr(refcycle_peers) = new_set;
    }
    handle peers = a.attr(refcycle_peers);
    // Ensure the proper ref count on the `peers` set. If it is > 1, the
    // objects will live forever. If it is < 1, the cycle will just be deleted
    // immediately.
    DRAKE_DEMAND(Py_REFCNT(peers.ptr()) == 1);
    PySet_Add(peers.ptr(), b.ptr());
  };
  make_link(p0, p1);
  make_link(p1, p0);
}

void check_and_make_ref_cycle(size_t peer0, handle p0, size_t peer1, handle p1,
    std::function<std::string(size_t)> not_gc_message_function) {
  // Returns false if the handle's value is None. Throws if the handle's value
  // is not of a garbage-collectable type.
  auto check_handle = [&](size_t n, handle p) -> bool {
    if (p.is_none()) {
      return false;
    }
    // Among the reasons the following check may fail is that one of the
    // participating pybind11::class_ types does not declare
    // pybind11::dynamic_attr().
    if (!PyType_IS_GC(Py_TYPE(p.ptr()))) {
      py::pybind11_fail(not_gc_message_function(n));
    }
    return true;
  };
  if (!check_handle(peer0, p0) || !check_handle(peer1, p1)) {
    // At least one of the handles is None. We can't construct a ref-cycle, but
    // neither should we complain. None variable values happen for any number of
    // legitimate reasons; appearance of None doesn't imply a defective use of
    // the ref_cycle policy.
    return;
  }
  make_ref_cycle(p0, p1);
}

}  // namespace

void ref_cycle_impl(
    size_t peer0, size_t peer1, const function_call& call, handle ret) {
  // Returns the handle selected by the given index. Throws if the index is
  // invalid.
  auto get_arg = [&](size_t n) -> handle {
    if (n == 0) {
      return ret;
    }
    if (n == 1 && call.init_self) {
      return call.init_self;
    }
    if (n <= call.args.size()) {
      return call.args[n - 1];
    }
    py::pybind11_fail(fmt::format(
        "Could not activate ref_cycle: index {} is invalid for function '{}'",
        n, call.func.name));
  };
  handle p0 = get_arg(peer0);
  handle p1 = get_arg(peer1);

  auto not_gc_error = [&call](size_t n) -> std::string {
    return fmt::format(
        "Could not activate ref_cycle: object type at index {} for "
        "function '{}' is not tracked by garbage collection.  Was the object "
        "defined with `pybind11::class_<...>(... pybind11::dynamic_attr())`?",
        n, call.func.name);
  };
  check_and_make_ref_cycle(peer0, p0, peer1, p1, not_gc_error);
}

void make_arbitrary_ref_cycle(
    handle p0, handle p1, const std::string& location_hint) {
  auto not_gc_error = [&location_hint](size_t n) -> std::string {
    return fmt::format(
        "Could not activate arbitrary ref_cycle: object type at argument {} "
        "for binding at '{}' is not tracked by garbage collection.  Was the "
        "object defined with `pybind11::class_<...>(... "
        "pybind11::dynamic_attr())`?",
        n, location_hint);
  };
  check_and_make_ref_cycle(0, p0, 1, p1, not_gc_error);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
