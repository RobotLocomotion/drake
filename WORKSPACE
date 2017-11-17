# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# http://bazel.io/ .

# To temporarily use a local copy of a github_archive, within this file add a
# local_repository_archive argument to its github_archive macro call, e.g.:
#
# github_archive(
#     name = "foobar",
#     local_repository_override = "/path/to/local/foo/bar",
#     repository = "foo/bar",
#     commit = "0123456789abcdef0123456789abcdef01234567",
#     sha256 = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",  # noqa
# )

workspace(name = "drake")

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")
load("//tools/workspace:github.bzl", "github_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

local_repository(
    name = "kythe",
    # TODO(jwnimmer-tri) According to Bazel documentation, the `path` argument
    # to `local_repository()` is supposed be an absolute path.  We should be
    # using using the __workspace_dir__ prefix here, like the other third_party
    # workspace items below.  However, doing so causes loading-phase errors.
    # We suspect that those errors are due to a Bazel bug, possibly related to
    # either bazelbuild/bazel#2811 and/or bazelbuild/bazel#269.  Since the only
    # use of kythe tooling is for pkg_config_package, and we plan to implement
    # our own version of that soon anyway, we'll leave this alone for now,
    # rather than diagnosing the errors.
    path = "third_party/com_github_google_kythe",
)

new_local_repository(
    name = "tinydir",
    build_file = "tools/workspace/tinydir/tinydir.BUILD.bazel",
    path = __workspace_dir__ + "/third_party/com_github_cxong_tinydir",
)

new_local_repository(
    name = "spruce",
    build_file = "tools/workspace/spruce/spruce.BUILD.bazel",
    path = __workspace_dir__ + "/third_party/josephdavisco_spruce",
)

new_local_repository(
    name = "stx",
    build_file = "tools/workspace/stx/stx.BUILD.bazel",
    path = __workspace_dir__ + "/third_party/com_github_tcbrindle_cpp17_headers",  # noqa
)

load("@kythe//tools/build_rules/config:pkg_config.bzl", "pkg_config_package")

pkg_config_package(
    name = "glib",
    modname = "glib-2.0",
)

pkg_config_package(
    name = "gthread",
    modname = "gthread-2.0",
)

load("//tools/workspace/python:python.bzl", "python_repository")

python_repository(
    name = "python",
    version = "2.7",
)

load("//tools/workspace/numpy:numpy.bzl", "numpy_repository")

numpy_repository(
    name = "numpy",
    python_version = "2.7",
)

github_archive(
    name = "gtest",
    repository = "google/googletest",
    commit = "release-1.8.0",
    sha256 = "58a6f4277ca2bc8565222b3bbd58a177609e9c488e8a72649359ba51450db7d8",  # noqa
    build_file = "tools/workspace/gtest/gtest.BUILD.bazel",
)

# When updating the version of gflags, update tools/install/gflags/gflags.cps
github_archive(
    name = "com_github_gflags_gflags",
    repository = "gflags/gflags",
    commit = "95ffb27c9c7496ede1409e042571054c70cb9519",
    sha256 = "723c21f783c720c0403c9b44bf500d1961a08bd2635cbc117107af22d2e1643f",  # noqa
)

github_archive(
    name = "styleguide",
    repository = "RobotLocomotion/styleguide",
    commit = "8d38c5909a5ab38824d7f4566b3f3c6ae4557826",
    sha256 = "09baa2280a63b9d2efe5c07d08f4674339b5b2a0424f71c429c33ac1783a4cd8",  # noqa
    build_file = "tools/workspace/styleguide/styleguide.BUILD.bazel",  # noqa
)

github_archive(
    name = "pycodestyle",
    repository = "PyCQA/pycodestyle",
    commit = "2.3.1",
    sha256 = "e9fc1ca3fd85648f45c0d2e33591b608a17d8b9b78e22c5f898e831351bacb03",  # noqa
    build_file = "tools/workspace/pycodestyle/pycodestyle.BUILD.bazel",
)

bitbucket_archive(
    name = "eigen",
    repository = "eigen/eigen",
    # N.B. See #5785; do your best not to have to bump this to a newer commit.
    commit = "3.3.3",
    sha256 = "94878cbfa27b0d0fbc64c00d4aafa137f678d5315ae62ba4aecddbd4269ae75f",  # noqa
    strip_prefix = "eigen-eigen-67e894c6cd8f",
    build_file = "tools/workspace/eigen/eigen.BUILD.bazel",
)

github_archive(
    name = "spdlog",
    repository = "gabime/spdlog",
    commit = "v0.13.0",
    sha256 = "d798a6ca19165f0a18a43938859359269f5a07fd8e0eb83ab8674739c9e8f361",  # noqa
    build_file = "tools/workspace/spdlog/spdlog.BUILD.bazel",
)

github_archive(
    name = "fmt",
    repository = "fmtlib/fmt",
    commit = "3.0.1",
    sha256 = "dce62ab75a161dd4353a98364feb166d35e7eea382169d59d9ce842c49c55bad",  # noqa
    build_file = "tools/workspace/fmt/fmt.BUILD.bazel",
)

MAVEN_REPOSITORY = "https://jcenter.bintray.com"

# In the unlikely event that you update the version here, verify that the
# licenses in tools/third_party/jchart2d/LICENSE are still applicable.
maven_jar(
    name = "net_sf_jchart2d_jchart2d",
    artifact = "net.sf.jchart2d:jchart2d:3.3.2",
    repository = MAVEN_REPOSITORY,
    sha1 = "4950821eefe4c204903e68b4d45a558b5ebdd6fa",
)

maven_jar(
    name = "com_jidesoft_jide_oss",
    artifact = "com.jidesoft:jide-oss:2.9.7",
    repository = MAVEN_REPOSITORY,
    sha1 = "a9bb0d8384012c25c1519f6dd9adc80dd720a050",
)

maven_jar(
    name = "commons_io_commons_io",
    artifact = "commons-io:commons-io:1.3.1",
    repository = MAVEN_REPOSITORY,
    sha1 = "b90b6ac57cf27a2858eaa490d02ba7945d18ca7b",
)

maven_jar(
    name = "org_apache_xmlgraphics_xmlgraphics_commons",
    artifact = "org.apache.xmlgraphics:xmlgraphics-commons:1.3.1",
    repository = MAVEN_REPOSITORY,
    sha1 = "f7d0fa54e2750acd82b1a241c043be6fce1bf0dc",
)

github_archive(
    name = "lcm",
    repository = "lcm-proj/lcm",
    commit = "87866bd0dbb1f9d5a0f662a6f5caecf469fd42d2",
    sha256 = "fd0afaf29954c26a725626b7bd24e873e303e84bb62dfcc05162be3f5ae30cd1",  # noqa
    build_file = "tools/workspace/lcm/lcm.BUILD.bazel",
)

# In the unlikely event that you update the version here, verify that the
# licenses in tools/third_party/libbot/ldpc LICENSE are still applicable.
github_archive(
    name = "libbot",
    repository = "RobotLocomotion/libbot2",
    commit = "495ae366d5e380b58254368217fc5c798e72aadd",
    sha256 = "c463460a4dd6133d6d21e6ab6e493fdcdca442d2df86bcb56749f6740bc61db5",  # noqa
    build_file = "tools/workspace/libbot/libbot.BUILD.bazel",
)

github_archive(
    name = "bullet",
    repository = "bulletphysics/bullet3",
    commit = "2.86.1",
    sha256 = "c058b2e4321ba6adaa656976c1a138c07b18fc03b29f5b82880d5d8228fbf059",  # noqa
    build_file = "tools/workspace/bullet/bullet.BUILD.bazel",
)

github_archive(
    name = "ccd",
    repository = "danfis/libccd",
    commit = "v2.0",
    sha256 = "1b4997e361c79262cf1fe5e1a3bf0789c9447d60b8ae2c1f945693ad574f9471",  # noqa
    build_file = "tools/workspace/ccd/ccd.BUILD.bazel",
)

github_archive(
    name = "octomap",
    repository = "OctoMap/octomap",
    commit = "v1.7.2",
    sha256 = "fe55efbb9ebf2b3388860e54b1c8a53d23e5a05de5956c043278013e01066c34",  # noqa
    build_file = "tools/workspace/octomap/octomap.BUILD.bazel",
)

github_archive(
    name = "fcl",
    repository = "flexible-collision-library/fcl",
    commit = "06d48b3b6f3605b8caf119d5208d8156eb64fe0d",
    sha256 = "da86ed593a908d075657a305abec1670b895278a99ba76632b7afb6e678a9978",  # noqa
    build_file = "tools/workspace/fcl/fcl.BUILD.bazel",
)

pkg_config_package(
    name = "ipopt",
    modname = "ipopt",
)

pkg_config_package(
    name = "nlopt",
    modname = "nlopt",
)

github_archive(
    name = "optitrack_driver",
    repository = "RobotLocomotion/optitrack-driver",
    commit = "b0d633570966e08b8915dee0867747596839d06c",
    sha256 = "5f7f46273f36073dc15191fe37dc538b4b23eaeaae63de153abeaa61d1134ad6",  # noqa
)

github_archive(
    name = "pybind11",
    repository = "RobotLocomotion/pybind11",
    commit = "ffcf754ae9e766632610975d22372a86a7b63014",
    sha256 = "7cd6f4efb02bf9ae17eeb2afba68023af913e61ae76e8b4254203d0eec019525",  # noqa
    build_file = "tools/workspace/pybind11/pybind11.BUILD.bazel",
)

github_archive(
    name = "lcmtypes_bot2_core",
    repository = "openhumanoids/bot_core_lcmtypes",
    commit = "99676541398749c2aab4b5b2c38be77d268085cc",
    sha256 = "896fd3edf87c7dfaae378af12d52d233577cc495ae96b5076c48b5b9ca700b4a",  # noqa
    build_file = "tools/workspace/lcmtypes_bot2_core/lcmtypes_bot2_core.BUILD.bazel",  # noqa
)

github_archive(
    name = "lcmtypes_robotlocomotion",
    repository = "RobotLocomotion/lcmtypes",
    commit = "8aea7a94d53dea01bfceba5f3cbe8e8cc9fb0244",
    sha256 = "f23a143d7865ea4f6cd9aeb2211fe36e20712a39d439cf16fea2b11685f29b61",  # noqa
    build_file = "tools/workspace/lcmtypes_robotlocomotion/lcmtypes_robotlocomotion.BUILD.bazel",  # noqa
)

pkg_config_package(
    name = "blas",
    modname = "blas",
)

pkg_config_package(
    name = "lapack",
    modname = "lapack",
)

github_archive(
    name = "scs",
    repository = "cvxgrp/scs",
    commit = "v1.2.6",
    sha256 = "b4bebb43a1257b6e88a5f97c855c0559d6c8a8c0548d3156fc5a28d82bb9533f",  # noqa
    build_file = "tools/workspace/scs/scs.BUILD.bazel",
)

github_archive(
    name = "tinyobjloader",
    repository = "syoyo/tinyobjloader",
    commit = "v1.0.6",
    sha256 = "19ee82cd201761954dd833de551edb570e33b320d6027e0d91455faf7cd4c341",  # noqa
    build_file = "tools/workspace/tinyobjloader/tinyobjloader.BUILD.bazel",
)

github_archive(
    name = "yaml_cpp",
    repository = "jbeder/yaml-cpp",
    commit = "85af926ddc5f3c8fb438001743e65ec3a039ceec",
    sha256 = "907fb42a502e1448a73959f9a648771b070d6d8513f16d74149f775fc56550ef",  # noqa
    build_file = "tools/workspace/yaml_cpp/yaml_cpp.BUILD.bazel",
)

load("//tools/workspace/buildifier:buildifier.bzl", "buildifier_repository")

buildifier_repository(
    name = "buildifier",
)

load("//tools/workspace/gurobi:gurobi.bzl", "gurobi_repository")

gurobi_repository(
    name = "gurobi",
)

load("//tools/workspace/mosek:mosek.bzl", "mosek_repository")

mosek_repository(
    name = "mosek",
)

git_repository(
    name = "snopt",
    remote = "git@github.com:RobotLocomotion/snopt.git",
    commit = "2ec980370eeb72897135b11570033a19bda885a7",
)

# Python Libraries
load("//tools/workspace:pypi.bzl", "pypi_archive")

pypi_archive(
    name = "six_archive",
    package = "six",
    version = "1.10.0",
    sha256 = "105f8d68616f8248e24bf0e9372ef04d3cc10104f1980f54d57b2ce73a5ad56a",  # noqa
    build_file = "tools/workspace/six/six.BUILD.bazel",
)

bind(
    name = "six",
    actual = "@six_archive//:six",
)

# When updating the version of protobuf,
# update tools/install/protobuf/protobuf.cps
github_archive(
    name = "protobuf",
    repository = "google/protobuf",
    commit = "v3.1.0",
    sha256 = "fb2a314f4be897491bb2446697be693d489af645cb0e165a85e7e64e07eb134d",  # noqa
)

pypi_archive(
    name = "semantic_version",
    version = "2.6.0",
    sha256 = "2a4328680073e9b243667b201119772aefc5fc63ae32398d6afafff07c4f54c0",  # noqa
    strip_prefix = "semantic_version",
    build_file = "tools/workspace/semantic_version/semantic_version.BUILD.bazel",  # noqa
)

github_archive(
    name = "pycps",
    repository = "mwoehlke/pycps",
    commit = "a6110cf2e769e9ff262a98ed18506ad565a14e89",
    sha256 = "62b5054705152ba971a6e9a358bfcc1359eca6f3ba8e5788befd82d606933d98",  # noqa
    build_file = "tools/workspace/pycps/pycps.BUILD.bazel",
)

# The "@python_headers//:python_headers" target is required by protobuf
# during "bazel query" but not "bazel build", so a stub is fine.
new_local_repository(
    name = "python_headers",
    path = "not/real/stub",
    build_file_content = ("cc_library(name = 'python_headers', " +
                          "visibility = ['//visibility:public'])"),
)

# If updating ignition_math version, do not forget to also update
# tools/ignition_math.BUILD in which the version number is hard-coded
# to configure config.hh with cmake_configure_file().
bitbucket_archive(
    name = "ignition_math",
    repository = "ignitionrobotics/ign-math",
    commit = "ignition-math3_3.2.0",
    sha256 = "1948c1610fa4403bce7ba2a262a29662990ee66aab00882411a0868afe0e5309",  # noqa
    strip_prefix = "ignitionrobotics-ign-math-e86e5bb392e4",
    build_file = "tools/workspace/ignition_math/ignition_math.BUILD.bazel",
)

bitbucket_archive(
    name = "ignition_rndf",
    repository = "ignitionrobotics/ign-rndf",
    commit = "ignition-rndf_0.1.5",
    sha256 = "fa1033be146ff51f3b2c679ff160838c1e3ca736c565b19510a5c9b6d352fbaf",  # noqa
    strip_prefix = "ignitionrobotics-ign-rndf-214a333fbdcb",
    build_file = "tools/workspace/ignition_rndf/ignition_rndf.BUILD.bazel",
)

load("//tools/workspace/boost:boost.bzl", "boost_repository")

boost_repository(
    name = "boost",
)

bitbucket_archive(
    name = "sdformat",
    repository = "osrf/sdformat",
    commit = "bac3dfb42cc7",
    sha256 = "b10a3ac68ed46f8d5780ddc687e6c89c71cb4c1e4e65449197f8aac76be903d8",  # noqa
    strip_prefix = "osrf-sdformat-bac3dfb42cc7",
    build_file = "tools/workspace/sdformat/sdformat.BUILD.bazel",
)

load("//tools/workspace/vtk:vtk.bzl", "vtk_repository")

vtk_repository(
    name = "vtk",
)

load("//tools/workspace/expat:expat.bzl", "expat_repository")

expat_repository(
    name = "expat",
)

pkg_config_package(
    name = "glew",
    modname = "glew",
)

pkg_config_package(
    name = "liblz4",
    modname = "liblz4",
)

pkg_config_package(
    name = "libpng",
    modname = "libpng",
)

pkg_config_package(
    name = "tinyxml",
    modname = "tinyxml",
)

pkg_config_package(
    name = "tinyxml2",
    modname = "tinyxml2",
)

load("//tools/workspace/zlib:zlib.bzl", "zlib_repository")

zlib_repository(
    name = "zlib",
)

load(
    "//tools/workspace/drake_visualizer:drake_visualizer.bzl",
    "drake_visualizer_repository",
)

drake_visualizer_repository(
    name = "drake_visualizer",
)
