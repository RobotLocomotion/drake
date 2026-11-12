#pragma once

/* This file declares helpers for storing a Python callable in a
CollisionCheckerParams::configuration_distance_function, and for handing that
callable over to a CollisionChecker without creating a reference cycle that
Python's garbage collector cannot see. */

#include <memory>
#include <string>
#include <utility>

#include <Eigen/Dense>

#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/collision_checker_params.h"

namespace drake {
namespace pydrake {
namespace internal {

/* The Python callable that a CollisionCheckerParams stores in its
`configuration_distance_function` (wrapped in a std::function). Unlike the
std::function conversion that pybind11 and nanobind provide, we can recover the
callable from it, which PrepareParamsForChecker() needs.

An "owning" instance holds a strong reference to the callable. A "borrowing"
instance holds only a raw pointer; some other object must keep the callable
alive. */
class __attribute__((visibility("hidden"))) PyDistanceFunction {
 public:
  static PyDistanceFunction Owning(py::object callable) {
    PyDistanceFunction result(callable.ptr());
    // The last copy may be destroyed without the GIL held.
    result.owner_ = std::shared_ptr<py::object>(
        new py::object(std::move(callable)), [](py::object* p) {
          py::gil_scoped_acquire guard;
          delete p;
        });
    return result;
  }

  static PyDistanceFunction Borrowing(py::handle callable) {
    return PyDistanceFunction(callable.ptr());
  }

  double operator()(
      const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) const {
    py::gil_scoped_acquire guard;
    py::handle callback = callable_;
    return py::cast<double>(callback(q1, q2));
  }

  py::handle callable() const { return callable_; }

  bool owns_callable() const { return owner_ != nullptr; }

 private:
  explicit PyDistanceFunction(PyObject* callable) : callable_(callable) {}

  PyObject* callable_{};
  std::shared_ptr<py::object> owner_;
};

/* Returns a copy of `*params` that is suitable for giving to a
CollisionChecker. If the params hold a Python distance function, the copy
borrows it, and `params_py` (the Python object for `*params`) is linked to it
so that Python's garbage collector can see it. In that case `*params` is also
changed to borrow the callable, so that it no longer hides a strong reference
to it. The caller must also ensure that the checker keeps `params_py` alive
(e.g., by using ensure_callback_lifetime). */
inline planning::CollisionCheckerParams PrepareParamsForChecker(
    py::handle params_py, planning::CollisionCheckerParams* params,
    const std::string& location_hint) {
  const auto* function =
      params->configuration_distance_function.target<PyDistanceFunction>();
  if (function != nullptr) {
    py::handle callable = function->callable();
    // This link is what keeps the callable alive once `*params` no longer
    // owns it.
    make_arbitrary_ref_link(params_py, callable, location_hint);
    if (function->owns_callable()) {
      params->configuration_distance_function =
          PyDistanceFunction::Borrowing(callable);
    }
  }
  return *params;
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
