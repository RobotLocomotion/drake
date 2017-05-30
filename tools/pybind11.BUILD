# -*- python -*-

load("@//tools:install.bzl", "install", "install_files")
load("@//tools:python_lint.bzl", "python_lint")

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

# TODO(jamiesnape): Refactor the install logic repeated for multiple external
# targets.

py_binary(
    name = "create-cps",
    srcs = ["@//tools:pybind11-create-cps.py"],
    main = "@//tools:pybind11-create-cps.py",
    visibility = ["//visibility:private"],
)

genrule(
    name = "cps",
    srcs = ["include/pybind11/common.h"],
    outs = ["pybind11.cps"],
    cmd = "$(location :create-cps) \"$<\" > \"$@\"",
    tools = [":create-cps"],
    visibility = ["//visibility:private"],
)

genrule(
    name = "cmake_exports",
    srcs = ["pybind11.cps"],
    outs = ["pybind11Config.cmake"],
    cmd = "$(location @pycps//:cps2cmake_executable) \"$<\" > \"$@\"",
    tools = ["@pycps//:cps2cmake_executable"],
    visibility = ["//visibility:private"],
)

genrule(
    name = "cmake_package_version",
    srcs = ["pybind11.cps"],
    outs = ["pybind11ConfigVersion.cmake"],
    cmd = "$(location @pycps//:cps2cmake_executable) --version-check \"$<\" > \"$@\"",
    tools = ["@pycps//:cps2cmake_executable"],
    visibility = ["//visibility:private"],
)

install_files(
    name = "install_cmake",
    dest = "lib/cmake/pybind11",
    files = [
        "pybind11Config.cmake",
        "pybind11ConfigVersion.cmake",
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
    deps = [":install_cmake"],
)

python_lint()
