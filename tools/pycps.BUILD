genrule(
  name = "copy_cps2cmake",
  srcs = ["cps2cmake"],
  outs = ["cps2cmake.py"],
  cmd = "cp \"$<\" \"$(@)\"",
)

py_library(
  name = "pycps",
  srcs = ["cps.py"],
  deps = ["@semantic_version//:semantic_version"]
)

py_binary(
  name = "cps2cmake_executable",
  srcs = ["cps2cmake.py"],
  main = "cps2cmake.py",
  deps = [":pycps"],
  visibility = ["//visibility:public"],
)
