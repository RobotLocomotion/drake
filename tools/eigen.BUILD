# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)
load("@drake//tools:python_lint.bzl", "python_lint")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "eigen",
    hdrs = glob(
        include = [
            "Eigen/*",
            "Eigen/**/*.h",
            "unsupported/Eigen/*",
            "unsupported/Eigen/**/*.h",
        ],
        exclude = ["**/CMakeLists.txt"],
    ),
    defines = ["EIGEN_MPL2_ONLY"],
    includes = ["."],
)

cmake_config(
    package = "Eigen3",
    script = "@drake//tools:eigen-create-cps.py",
    version_file = "Eigen/src/Core/util/Macros.h",
)

install_cmake_config(package = "Eigen3")  # Creates rule :install_cmake_config.

install(
    name = "install",
    doc_dest = "share/doc/eigen3",
    docs = glob(["COPYING.*"]),
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/eigen3",
    targets = [":eigen"],
    deps = [":install_cmake_config"],
)

python_lint()
