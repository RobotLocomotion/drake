cc_library(
    name = "control_utils",
    srcs = [],
    hdrs = ["control_utils.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/math:geometric_transform",
    ],
)

cc_library(
    name = "example_qp_input_for_valkyrie",
    srcs = ["example_qp_input_for_valkyrie.cc"],
    hdrs = ["example_qp_input_for_valkyrie.h"],
    linkstatic = 1,
    deps = [
        ":qp_controller",
    ],
)

cc_library(
    name = "rigid_body_tree_utils",
    srcs = ["rigid_body_tree_utils.cc"],
    hdrs = ["rigid_body_tree_utils.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/robotInterfaces:side",
    ],
)

cc_library(
    name = "humanoid_status",
    srcs = ["humanoid_status.cc"],
    hdrs = ["humanoid_status.h"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree_utils",
        "//drake/common",
    ],
)

cc_library(
    name = "qp_controller",
    srcs = ["qp_controller.cc"],
    hdrs = ["qp_controller.h"],
    linkstatic = 1,
    deps = [
        ":humanoid_status",
        "//drake/common",
        "//drake/math:gradient",
        "//drake/solvers:mathematical_program",
    ],
)

#test/
cc_test(
    name = "valkyrie_balancing_test",
    srcs = ["test/valkyrie_balancing_test.cc"],
    data = [
        "//drake/examples/Valkyrie:models",
    ],
    tags = [
        "exclusive",
        "gurobi",
        "local",
    ],
    deps = [
        ":control_utils",
        ":example_qp_input_for_valkyrie",
        ":qp_controller",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
        "@gtest//:main",
    ],
)
