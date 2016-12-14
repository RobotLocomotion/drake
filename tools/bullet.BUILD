# -*- python -*-

# Note that this is only a portion of Bullet.
cc_library(
    name = "lib",
    srcs = glob([
        "src/BulletCollision/**/*.cpp",
        "src/LinearMath/**/*.cpp",
    ]),
    hdrs = glob([
        "src/BulletCollision/**/*.h",
        "src/LinearMath/**/*.h",
    ]) + ["src/btBulletCollisionCommon.h"],
    copts = ["-Wno-all"],
    defines = ["BT_USE_DOUBLE_PRECISION"],
    includes = ["src"],
    visibility = ["//visibility:public"],
)
