#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

using pybind11::handle;
using pybind11::detail::function_call;

namespace drake {
namespace pydrake {
namespace internal {

void do_ref_cycle_impl(handle p0, handle p1) {
  if (!p0 || !p1) {
    pybind11::pybind11_fail("Could not activate ref_cycle!");
  }

  if (p1.is_none() || p0.is_none()) {
    return;  // Nothing to do; at least one peer is missing.
  }
  // Among the reasons the following checks may fail is that one of the
  // participating pybind11::class_ types does not declare
  // pybind11::dynamic_attr().
  DRAKE_THROW_UNLESS(PyType_IS_GC(Py_TYPE(p0.ptr())));
  DRAKE_THROW_UNLESS(PyType_IS_GC(Py_TYPE(p1.ptr())));

  // Each peer will have a new/updated attribute, containing a set of
  // handles. Insert each into the other's handle set. Create the set first
  // if it is not yet existing.
  auto make_link = [](handle a, handle b) {
    static const char refcycle_peers[] = "_pydrake_ref_cycle_peers";
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

void ref_cycle_impl(
    size_t Peer0, size_t Peer1, const function_call& call, handle ret) {
  auto get_arg = [&](size_t n) {
    if (n == 0) {
      return ret;
    }
    if (n == 1 && call.init_self) {
      return call.init_self;
    }
    if (n <= call.args.size()) {
      return call.args[n - 1];
    }
    return handle();
  };

  do_ref_cycle_impl(get_arg(Peer0), get_arg(Peer1));
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
