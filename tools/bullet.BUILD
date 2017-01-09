# -*- python -*-

# These options are used when building Bullet C/C++ code.
# They do not flow downstream to Bullet-using libraries.
BULLET_COPTS = [
    "-Wno-all",
    "-Wno-extra",
    "-Wno-overloaded-virtual",
]

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
    copts = BULLET_COPTS,
    defines = ["BT_USE_DOUBLE_PRECISION"],
    includes = ["src"],
    visibility = ["//visibility:public"],
)
