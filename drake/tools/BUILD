load("@protobuf//:protobuf.bzl", "py_proto_library")

py_proto_library(
    name = "named_vector",
    srcs = ["named_vector.proto"],
)

py_binary(
    name = "lcm_vector_gen",
    srcs = ["lcm_vector_gen.py"],
    deps = [
        ":named_vector",
    ],
)
