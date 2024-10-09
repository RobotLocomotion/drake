#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

using pybind11::handle;
using pybind11::detail::function_call;

namespace drake {
namespace pydrake {
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
    handle peers;
    if (hasattr(a, refcycle_peers)) {
      peers = a.attr(refcycle_peers);
    } else {
      peers = PySet_New(nullptr);
      DRAKE_DEMAND(PyType_IS_GC(Py_TYPE(peers.ptr())));
      a.attr(refcycle_peers) = peers;
      Py_DECREF(peers.ptr());
    }
    // Ensure the proper ref count on the `peers` set. If it is > 1, the
    // objects will live forever. If it is < 1, the cycle will just be deleted
    // immediately.
    DRAKE_DEMAND(Py_REFCNT(peers.ptr()) == 1);
    PySet_Add(peers.ptr(), b.ptr());
  };
  make_link(p0, p1);
  make_link(p1, p0);
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
    pybind11::pybind11_fail(fmt::format(
        "Could not activate ref_cycle: index {} is invalid for function '{}'",
        n, call.func.name));
  };
  handle p0 = get_arg(peer0);
  handle p1 = get_arg(peer1);

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
      pybind11::pybind11_fail(fmt::format(
          "Could not activate ref_cycle: object type at index {} for "
          "function '{}' is not tracked by garbage collection.",
          n, call.func.name));
    }
    return true;
  };
  if (!check_handle(peer0, p0) || !check_handle(peer1, p1)) {
    // At least one of the handles is None. We can't construct a ref-cycle, but
    // neither should we complain. A None variable value could happen for any
    // number of legitimate reasons, and does not mean that the ref_cycle call
    // policy is defective.
    return;
  }
  make_ref_cycle(p0, p1);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
