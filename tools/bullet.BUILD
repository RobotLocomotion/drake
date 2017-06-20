# -*- python -*-

load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")
load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)
load("@drake//tools:python_lint.bzl", "python_lint")

package(default_visibility = ["//visibility:public"])

# Note that this is only a portion of Bullet.

# Keep defines in sync with Components.LinearMath.Definitions in
# bullet-create-cps.py.
cc_library(
    name = "LinearMath",
    srcs = glob(["src/LinearMath/**/*.cpp"]),
    hdrs = glob(["src/LinearMath/**/*.h"]),
    copts = ["-Wno-all"],
    defines = ["BT_USE_DOUBLE_PRECISION"],
    includes = ["src"],
)

cc_library(
    name = "BulletCollision",
    srcs = glob(["src/BulletCollision/**/*.cpp"]),
    hdrs = glob(["src/BulletCollision/**/*.h"]) + [
        "src/btBulletCollisionCommon.h",
    ],
    copts = ["-Wno-all"],
    includes = ["src"],
    deps = [":LinearMath"],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = ["LICENSE.txt"],
    mode = "0644",
    package_dir = "bullet",
)

cmake_config(
    package = "Bullet",
    script = "@drake//tools:bullet-create-cps.py",
    version_file = "VERSION",
)

install_cmake_config(package = "Bullet")  # Creates rule :install_cmake_config.

install(
    name = "install",
    doc_dest = "share/doc/bullet",
    docs = ["AUTHORS.txt"],
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/bullet",
    hdr_strip_prefix = ["src"],
    license_docs = ["LICENSE.txt"],
    targets = [
        ":BulletCollision",
        ":LinearMath",
    ],
    deps = [":install_cmake_config"],
)

python_lint()
