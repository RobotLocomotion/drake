#pragma once

#include "drake/bindings/pybind11_ext/numpy_dtypes_user.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

/// Defines implicit conversions from numeric types to a given user-defined
/// NumPy dtype.
template <typename Class>
void DefImplicitConversionsFromNumericTypes(py::dtype_user<Class>* cls) {
  py::module np = py::module::import("numpy");
  // TODO(eric.cousineau): Remove this once pybind/pybind11#1329 lands, and rely
  // on `py::dtype::of<int64_t>`.
  py::dtype np_int64 = py::reinterpret_borrow<py::dtype>(
      np.attr("dtype")(np.attr("int64")));
  // Due to how `np.result_type` works, it promotes data type based on the
  // scalar magnitude. For `np.allclose`, it uses `np.result_type(y, 1.)`, where
  // `1.` resolves to `np.float16`. We don't need to have a functioning
  // converter, but simply indicate that we *could* implicitly cast.
  py::dtype np_float16 = py::reinterpret_borrow<py::dtype>(
        np.attr("dtype")(np.attr("float16")));

  (*cls)
    // Upcasting can be implicit, especially for matrix multiplication.
    .def_loop(py::dtype_method::implicit_conversion<double, Class>())
    .def_loop(py::dtype_method::implicit_conversion<int, Class>())
    // `int64` is needed for implicitly converting arguments from
    // `np.array([<integers>])`, which by default resolves to this dtype.
    .def_loop(
        py::dtype_method::implicit_conversion<int64_t, Class>(), np_int64)
    .def_loop(
        py::dtype_method::implicit_conversion([](int16_t) -> Class {
            throw std::runtime_error(
                "No actual half float conversion available");
        }), np_float16)
    // See https://github.com/numpy/numpy/issues/10904 for next 2 casts.
    // NOLINTNEXTLINE(runtime/int): Use platform-dependent name for NumPy.
    .def_loop(py::dtype_method::implicit_conversion<long, Class>())
    .def_loop(py::dtype_method::implicit_conversion<bool, Class>());
}

}  // namespace pydrake
}  // namespace drake
