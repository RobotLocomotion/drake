# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# http://bazel.io/ .

workspace(name = "drake")

load("//tools/third_party/kythe/tools/build_rules/config:pkg_config.bzl", "pkg_config_package")

pkg_config_package(
    name = "glib",
    modname = "glib-2.0",
)

pkg_config_package(
    name = "python2",
    modname = "python2",
)

new_http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
    sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
    build_file = "tools/gtest.BUILD",
    strip_prefix = "googletest-release-1.7.0",
)

git_repository(
    name   = "gflags",
    commit = "a69b2544d613b4bee404988710503720c487119a",
    remote = "https://github.com/gflags/gflags.git"
)

new_git_repository(
    name = "google_styleguide",
    remote = "https://github.com/google/styleguide.git",
    commit = "159b4c81bbca97a9ca00f1195a37174388398a67",
    build_file = "tools/google_styleguide.BUILD",
)

new_git_repository(
    name = "eigen",
    remote = "https://github.com/RobotLocomotion/eigen-mirror.git",
    commit = "d3ee2bc648be3d8be8c596a9a0aefef656ff8637",
    build_file = "tools/eigen.BUILD",
)

new_git_repository(
    name = "spdlog",
    remote = "https://github.com/gabime/spdlog.git",
    commit = "43a4048b92ef5b7eff6dc637a621c7da3a41d194",
    build_file = "tools/spdlog.BUILD",
)

new_git_repository(
    name = "lcm",
    remote = "https://github.com/lcm-proj/lcm.git",
    commit = "755d8108bf4447d83786e0e6586875371ba859e5",
    build_file = "tools/lcm.BUILD",
)

new_git_repository(
    name = "bullet",
    remote = "https://github.com/RobotLocomotion/bullet3.git",
    commit = "ae2c4ca0618d55c6a29900aed75b958604149fdb",
    build_file = "tools/bullet.BUILD",
)

new_git_repository(
    name = "ccd",
    remote = "https://github.com/danfis/libccd.git",
    commit = "16b9379fb6e8610566fe5e1396166daf7106f165",
    build_file = "tools/ccd.BUILD",
)

new_git_repository(
    name = "octomap",
    remote = "https://github.com/OctoMap/octomap.git",
    commit = "6d7c31ae4df2c93cb8a954e44d442338b58d3558",
    build_file = "tools/octomap.BUILD",
)

new_git_repository(
    name = "fcl",
    remote = "https://github.com/flexible-collision-library/fcl.git",
    commit = "06d48b3b6f3605b8caf119d5208d8156eb64fe0d",
    build_file = "tools/fcl.BUILD",
)

# Necessary for buildifier.
http_archive(
    name = "io_bazel_rules_go",
    sha256 = "b7759f01d29c075db177f688ffb4464aad2b8fbb7017f89a1d3819ce07f1d584",
    strip_prefix = "rules_go-0.3.1",
    url = "https://github.com/bazelbuild/rules_go/archive/0.3.1.tar.gz",
)

# Necessary for buildifier.
load("@io_bazel_rules_go//go:def.bzl", "go_repositories")

# Necessary for buildifier.
go_repositories()

git_repository(
    name = "com_github_bazelbuild_buildifier",
    commit = "93929369232fcda305607a2e0aa7b3cd9cf8912d",
    remote = "https://github.com/bazelbuild/buildifier.git",
)

new_git_repository(
    name = "yaml_cpp",
    remote = "https://github.com/jbeder/yaml-cpp.git",
    commit = "85af926ddc5f3c8fb438001743e65ec3a039ceec",
    build_file = "tools/yaml_cpp.BUILD",
)

load("//tools:gurobi.bzl", "gurobi_repository")
gurobi_repository(
    name = "gurobi",
    workspace_dir = __workspace_dir__,
    build_file = "tools/gurobi.BUILD",
)

