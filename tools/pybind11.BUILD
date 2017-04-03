# -*- python -*-

cc_library(
    name = "pybind11",
    hdrs = [
        "include/pybind11/attr.h",
        "include/pybind11/cast.h",
        "include/pybind11/chrono.h",
        "include/pybind11/common.h",
        "include/pybind11/complex.h",
        "include/pybind11/descr.h",
        "include/pybind11/eigen.h",
        "include/pybind11/eval.h",
        "include/pybind11/functional.h",
        "include/pybind11/numpy.h",
        "include/pybind11/operators.h",
        "include/pybind11/options.h",
        "include/pybind11/pybind11.h",
        "include/pybind11/pytypes.h",
        "include/pybind11/stl.h",
        "include/pybind11/stl_bind.h",
        "include/pybind11/typeid.h",
    ],
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@eigen//:eigen",
        "@numpy//:numpy",
        "@python//:python",
    ],
)
