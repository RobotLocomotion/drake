load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

py_proto_compile(
    name = "named_vector",
    protos = [
        "named_vector.proto",
    ],
    with_grpc = False,
)

py_binary(
    name = "lcm_vector_gen",
    srcs = [
        "lcm_vector_gen.py",
        ":named_vector",
    ],
    deps = [
        "@protobuf_python//:protobuf_python",
    ],
)
