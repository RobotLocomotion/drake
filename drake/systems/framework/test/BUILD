package(default_visibility = ["//visibility:public"])

cc_test(
    name = "leaf_context_test",
    srcs = ["leaf_context_test.cc"],
    deps = [
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
    linkstatic=1,
    testonly=1)


cc_test(
    name = "modal_state_test",
    srcs = ["modal_state_test.cc"],
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
    linkstatic=1,
    testonly=1)


cc_test(
    name = "cache_test",
    srcs = ["cache_test.cc"],
    deps = [
        "//drake/common", 
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
    linkstatic=1,
    testonly=1)
