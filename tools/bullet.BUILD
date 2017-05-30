# -*- python -*-

load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")
load("@//tools:install.bzl", "install", "install_files")
load("@//tools:python_lint.bzl", "python_lint")

package(default_visibility = ["//visibility:public"])

# Note that this is only a portion of Bullet.
cc_library(
    name = "BulletCollision",
    srcs = glob(["src/BulletCollision/**/*.cpp"]),
    hdrs = glob(["src/BulletCollision/**/*.h"]) + ["src/btBulletCollisionCommon.h"],
    copts = ["-Wno-all"],
    defines = ["BT_USE_DOUBLE_PRECISION"],
    includes = ["src"],
    deps = [":LinearMath"],
)

cc_library(
    name = "LinearMath",
    srcs = glob(["src/LinearMath/**/*.cpp"]),
    hdrs = glob(["src/LinearMath/**/*.h"]),
    copts = ["-Wno-all"],
    defines = ["BT_USE_DOUBLE_PRECISION"],
    includes = ["src"],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = ["LICENSE.txt"],
    mode = "0644",
    package_dir = "bullet",
)

# TODO(jamiesnape): Refactor the install logic repeated for multiple external
# targets.

py_binary(
    name = "create-cps",
    srcs = ["@//tools:bullet-create-cps.py"],
    main = "@//tools:bullet-create-cps.py",
    visibility = ["//visibility:private"],
)

genrule(
    name = "cps",
    srcs = ["VERSION"],
    outs = ["bullet.cps"],
    cmd = "$(location :create-cps) \"$<\" > \"$@\"",
    tools = [":create-cps"],
    visibility = ["//visibility:private"],
)

genrule(
    name = "cmake_exports",
    srcs = ["bullet.cps"],
    outs = ["BulletConfig.cmake"],
    cmd = "$(location @pycps//:cps2cmake_executable) \"$<\" > \"$@\"",
    tools = ["@pycps//:cps2cmake_executable"],
    visibility = ["//visibility:private"],
)

genrule(
    name = "cmake_package_version",
    srcs = ["bullet.cps"],
    outs = ["BulletConfigVersion.cmake"],
    cmd = "$(location @pycps//:cps2cmake_executable) --version-check \"$<\" > \"$@\"",
    tools = ["@pycps//:cps2cmake_executable"],
    visibility = ["//visibility:private"],
)

install_files(
    name = "install_cmake",
    dest = "lib/cmake/bullet",
    files = [
        "BulletConfig.cmake",
        "BulletConfigVersion.cmake",
    ],
    strip_prefix = ["**/"],
)

install(
    name = "install",
    doc_dest = "share/doc/bullet",
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/bullet",
    docs = ["AUTHORS.txt"],
    license_docs = ["LICENSE.txt"],
    targets = [":BulletCollision", ":LinearMath"],
    deps = [":install_cmake"],
)

python_lint()
