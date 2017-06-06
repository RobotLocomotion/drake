# -*- python -*-

genrule(
    name = "copy_cps2cmake",
    srcs = ["cps2cmake"],
    outs = ["cps2cmake.py"],
    cmd = "cp \"$<\" \"$(@)\"",
)

# LGPL; don't use externally (okay for build-time stuff only)
py_library(
    name = "cps",
    srcs = ["cps.py"],
    visibility = ["//visibility:public"],
    deps = ["@semantic_version"],
)

py_binary(
    name = "cps2cmake_executable",
    srcs = ["cps2cmake.py"],
    main = "cps2cmake.py",
    visibility = ["//visibility:public"],
    deps = [":cps"],
)
