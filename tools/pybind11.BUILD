# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
    "install_files",
)
load("@drake//tools:python_lint.bzl", "python_lint")

package(default_visibility = ["//visibility:public"])

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
    deps = [
        "@eigen",
        "@numpy",
        "@python",
    ],
)

cmake_config(
    package = "pybind11",
    script = "@drake//tools:pybind11-create-cps.py",
    version_file = "include/pybind11/common.h",
)

# Creates rule :install_cmake_config.
install_cmake_config(package = "pybind11")

install_files(
    name = "install_extra_cmake",
    dest = "lib/cmake/pybind11",
    files = [
        "tools/FindNumPy.cmake",
        "tools/FindPythonLibsNew.cmake",
        "tools/pybind11Tools.cmake",
    ],
    strip_prefix = ["**/"],
)

install(
    name = "install",
    doc_dest = "share/doc/pybind11",
    guess_hdrs = "PACKAGE",
    hdr_strip_prefix = ["include"],
    license_docs = ["LICENSE"],
    targets = [":pybind11"],
    deps = [
        ":install_cmake_config",
        ":install_extra_cmake",
    ],
)

python_lint()
