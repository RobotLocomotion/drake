# -*- python -*-

genrule(
    name = "copy_cps2cmake",
    srcs = ["cps2cmake"],
    outs = ["cps2cmake.py"],
    cmd = "cp \"$<\" \"$(@)\"",
)

py_library(
    name = "cps",
    srcs = ["cps.py"],
    visibility = ["//visibility:private"],  # LGPL; don't use externally
    deps = ["@semantic_version"],
)

py_binary(
    name = "cps2cmake_executable",
    srcs = ["cps2cmake.py"],
    main = "cps2cmake.py",
    visibility = ["//visibility:public"],
    deps = [":cps"],
)
