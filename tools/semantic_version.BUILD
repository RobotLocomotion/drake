load("@//tools:drake.bzl", "drake_generate_file")

drake_generate_file(
    name = "init_genrule",
    out = "__init__.py",
    content = "from semantic_version import *",
)

py_library(
  name = "semantic_version",
  srcs = glob(["semantic_version/*.py"]) + ["__init__.py"],
  visibility = ["//visibility:public"],
)
