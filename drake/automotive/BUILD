# -*- python -*-

cc_library(
    name = "automotive",
    hdrs = [
        "create_trajectory_params.h",
        "curve2.h",
        "gen/driving_command.h",
        "gen/euler_floating_joint_state.h",
        "gen/idm_with_trajectory_agent_state.h",
        "gen/simple_car_config.h",
        "gen/simple_car_state.h",
        "idm_with_trajectory_agent.h",
        "idm_with_trajectory_agent-inl.h",
        "simple_car.h",
        "simple_car-inl.h",
        "trajectory_car.h",
    ],
    srcs = [
        "create_trajectory_params.cc",
        "curve2.cc",
        "gen/driving_command.cc",
        "gen/euler_floating_joint_state.cc",
        "gen/idm_with_trajectory_agent_state.cc",
        "gen/simple_car_config.cc",
        "gen/simple_car_state.cc",
        "idm_with_trajectory_agent.cc",
        "simple_car.cc",
        "trajectory_car.cc",
        ],
    visibility = ["//visibility:public"],
    deps = ["//drake/systems:systems_analysis"],
)
