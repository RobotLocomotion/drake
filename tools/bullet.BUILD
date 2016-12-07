# -*- python -*-

cc_library(
    name = "lib",
    hdrs = glob([
        "src/BulletCollision/**/*.h",
        "src/LinearMath/**/*.h",
    ]) + ["src/btBulletCollisionCommon.h"],
    srcs = glob([
        "src/BulletCollision/**/*.cpp",
        "src/LinearMath/**/*.cpp",
    ]),
    visibility = ["//visibility:public"],
    includes = ["src"],
    copts = ["-Wno-all"],
    defines = ["BT_USE_DOUBLE_PRECISION"],
)
