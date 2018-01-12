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
load("//tools/workspace:which.bzl", "which")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

github_archive(
    name = "tinydir",
    repository = "cxong/tinydir",
    commit = "3aae9224376b5e1a23fd824f19d9501162620b53",
    sha256 = "fa7eec0baaa5f6c57df1a38b064ec3a4f098f477f0a64d97c646a4470ffdd3b6",  # noqa
    build_file = "@drake//tools/workspace/tinydir:package.BUILD.bazel",
)

load("//tools/workspace/spruce:package.bzl", "spruce_repository")

spruce_repository(name = "spruce")

github_archive(
    name = "stx",
    repository = "tcbrindle/cpp17_headers",
    commit = "e416b34c132e05487ef8d7fd197f4513d6535c1d",
    sha256 = "26a534bef07e9e0b7c9bfa4dc0ae0b154e4a5ad8d71a2487f6e33d6e3ca0dadc",  # noqa
    build_file = "@drake//tools/workspace/stx:package.BUILD.bazel",
)

# This local repository imports the protobuf build rules for Bazel (based on
# the upstream protobuf.bzl build rules); in constrast, the protobuf runtime is
# loaded as @libprotobuf.
local_repository(
    name = "com_google_protobuf",
    # TODO(clalancette) Per https://github.com/RobotLocomotion/drake/pull/7361
    # this should use an absolute path (so this should be prepended by
    # __workspace_dir__).  However, in a clean build, this did not work.  We
    # should investigate that and fix it.
    path = "third_party/com_github_google_protobuf",
)

load("//tools/workspace/ibex:package.bzl", "ibex_repository")

ibex_repository(name = "ibex")

load("//tools/workspace/dreal:package.bzl", "dreal_repository")

dreal_repository(name = "dreal")

load("//tools/workspace/glib:package.bzl", "glib_repository")

glib_repository(name = "glib")

load("//tools/workspace/gthread:package.bzl", "gthread_repository")

gthread_repository(name = "gthread")

load("//tools/workspace/libprotobuf:package.bzl", "libprotobuf_repository")

libprotobuf_repository(name = "libprotobuf")

# Find the protoc binary on $PATH.
which(
    name = "protoc",
    command = "protoc",
)

load("//tools/workspace/python:package.bzl", "python_repository")

python_repository(name = "python")

load("//tools/workspace/numpy:package.bzl", "numpy_repository")

numpy_repository(name = "numpy")

github_archive(
    name = "gtest",
    repository = "google/googletest",
    commit = "release-1.8.0",
    sha256 = "58a6f4277ca2bc8565222b3bbd58a177609e9c488e8a72649359ba51450db7d8",  # noqa
    build_file = "@drake//tools/workspace/gtest:package.BUILD.bazel",
)

load("//tools/workspace/gflags:package.bzl", "gflags_repository")

gflags_repository(name = "gflags")

github_archive(
    name = "styleguide",
    repository = "RobotLocomotion/styleguide",
    commit = "f9fb031554d398431bc0efcb511102d41bbed089",
    sha256 = "1e40f4595406e208de8bde66bc3425e6c0dce4ea96254cc2c7e4105316df9a31",  # noqa
    build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
)

github_archive(
    name = "pycodestyle",
    repository = "PyCQA/pycodestyle",
    commit = "2.3.1",
    sha256 = "e9fc1ca3fd85648f45c0d2e33591b608a17d8b9b78e22c5f898e831351bacb03",  # noqa
    build_file = "@drake//tools/workspace/pycodestyle:package.BUILD.bazel",
)

bitbucket_archive(
    name = "eigen",
    repository = "eigen/eigen",
    # N.B. See #5785; do your best not to have to bump this to a newer commit.
    commit = "3.3.3",
    sha256 = "94878cbfa27b0d0fbc64c00d4aafa137f678d5315ae62ba4aecddbd4269ae75f",  # noqa
    strip_prefix = "eigen-eigen-67e894c6cd8f",
    build_file = "@drake//tools/workspace/eigen:package.BUILD.bazel",
)

github_archive(
    name = "spdlog",
    repository = "gabime/spdlog",
    commit = "v0.13.0",
    sha256 = "d798a6ca19165f0a18a43938859359269f5a07fd8e0eb83ab8674739c9e8f361",  # noqa
    build_file = "@drake//tools/workspace/spdlog:package.BUILD.bazel",
)

github_archive(
    name = "fmt",
    repository = "fmtlib/fmt",
    commit = "3.0.1",
    sha256 = "dce62ab75a161dd4353a98364feb166d35e7eea382169d59d9ce842c49c55bad",  # noqa
    build_file = "@drake//tools/workspace/fmt:package.BUILD.bazel",
)

load("//tools/workspace/net_sf_jchart2d:package.bzl", "net_sf_jchart2d_repository")  # noqa

net_sf_jchart2d_repository(name = "net_sf_jchart2d")

load("//tools/workspace/com_jidesoft_jide_oss:package.bzl", "com_jidesoft_jide_oss_repository")  # noqa

com_jidesoft_jide_oss_repository(name = "com_jidesoft_jide_oss")

load("//tools/workspace/commons_io:package.bzl", "commons_io_repository")

commons_io_repository(name = "commons_io")

load("//tools/workspace/org_apache_xmlgraphics_commons:package.bzl", "org_apache_xmlgraphics_commons_repository")  # noqa

org_apache_xmlgraphics_commons_repository(name = "org_apache_xmlgraphics_commons")  # noqa

github_archive(
    name = "lcm",
    repository = "lcm-proj/lcm",
    commit = "87866bd0dbb1f9d5a0f662a6f5caecf469fd42d2",
    sha256 = "fd0afaf29954c26a725626b7bd24e873e303e84bb62dfcc05162be3f5ae30cd1",  # noqa
    build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
)

github_archive(
    name = "bullet",
    repository = "bulletphysics/bullet3",
    commit = "2.86.1",
    sha256 = "c058b2e4321ba6adaa656976c1a138c07b18fc03b29f5b82880d5d8228fbf059",  # noqa
    build_file = "@drake//tools/workspace/bullet:package.BUILD.bazel",
)

github_archive(
    name = "ccd",
    repository = "danfis/libccd",
    commit = "5677d384315d64c41a9e1dabe6a531f10ffbb7fb",
    sha256 = "3b37ef4555d087f7abb6aa59c3b5cecb96410ea10e95a086ef2771569fb6fdfb",  # noqa
    build_file = "@drake//tools/workspace/ccd:package.BUILD.bazel",
)

github_archive(
    name = "octomap",
    repository = "OctoMap/octomap",
    commit = "v1.8.1",
    sha256 = "8b18ef7693e87f1400b9a8bc41f86e3b28259ac98c0b458037232652380aa6af",  # noqa
    build_file = "@drake//tools/workspace/octomap:package.BUILD.bazel",
)

github_archive(
    name = "fcl",
    repository = "flexible-collision-library/fcl",
    commit = "43048336c34a01156dc216e8534ffb2788675ddf",
    sha256 = "fd74916b92ed58e77c06097dc18f545462417daa8c96fa8ea2a5c81cd3205917",  # noqa
    build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
)

load("//tools/workspace/ipopt:package.bzl", "ipopt_repository")

ipopt_repository(name = "ipopt")

load("//tools/workspace/nlopt:package.bzl", "nlopt_repository")

nlopt_repository(name = "nlopt")

github_archive(
    name = "optitrack_driver",
    repository = "RobotLocomotion/optitrack-driver",
    commit = "b0d633570966e08b8915dee0867747596839d06c",
    sha256 = "5f7f46273f36073dc15191fe37dc538b4b23eaeaae63de153abeaa61d1134ad6",  # noqa
)

github_archive(
    name = "pybind11",
    repository = "RobotLocomotion/pybind11",
    commit = "48999b69bde29cdf8d616d4fbd3d6ab1c561027d",
    sha256 = "2ea18adfb608948cab1b5978081dc8c318ed47573ccd66f1603a37fbdbfc56da",  # noqa
    build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
)

github_archive(
    name = "lcmtypes_bot2_core",
    repository = "openhumanoids/bot_core_lcmtypes",
    commit = "99676541398749c2aab4b5b2c38be77d268085cc",
    sha256 = "896fd3edf87c7dfaae378af12d52d233577cc495ae96b5076c48b5b9ca700b4a",  # noqa
    build_file = "@drake//tools/workspace/lcmtypes_bot2_core:package.BUILD.bazel",  # noqa
)

github_archive(
    name = "lcmtypes_robotlocomotion",
    repository = "RobotLocomotion/lcmtypes",
    commit = "8aea7a94d53dea01bfceba5f3cbe8e8cc9fb0244",
    sha256 = "f23a143d7865ea4f6cd9aeb2211fe36e20712a39d439cf16fea2b11685f29b61",  # noqa
    build_file = "@drake//tools/workspace/lcmtypes_robotlocomotion:package.BUILD.bazel",  # noqa
)

load("//tools/workspace/blas:package.bzl", "blas_repository")

blas_repository(name = "blas")

load("//tools/workspace/lapack:package.bzl", "lapack_repository")

lapack_repository(name = "lapack")

github_archive(
    name = "scs",
    repository = "cvxgrp/scs",
    commit = "v1.2.6",
    sha256 = "b4bebb43a1257b6e88a5f97c855c0559d6c8a8c0548d3156fc5a28d82bb9533f",  # noqa
    build_file = "@drake//tools/workspace/scs:package.BUILD.bazel",
)

github_archive(
    name = "tinyobjloader",
    repository = "syoyo/tinyobjloader",
    commit = "v1.0.6",
    sha256 = "19ee82cd201761954dd833de551edb570e33b320d6027e0d91455faf7cd4c341",  # noqa
    build_file = "@drake//tools/workspace/tinyobjloader:package.BUILD.bazel",
)

load("//tools/workspace/yaml_cpp:package.bzl", "yaml_cpp_repository")

yaml_cpp_repository(name = "yaml_cpp")

load("//tools/workspace/buildifier:package.bzl", "buildifier_repository")

buildifier_repository(name = "buildifier")

load("//tools/workspace/gurobi:package.bzl", "gurobi_repository")

gurobi_repository(name = "gurobi")

load("//tools/workspace/mosek:package.bzl", "mosek_repository")

mosek_repository(name = "mosek")

# We directly declare a git_repository because the snopt source code requires
# authentication, and our github_archive does not (yet, easily) support that.
git_repository(
    name = "snopt",
    remote = "git@github.com:RobotLocomotion/snopt.git",
    commit = "0f475624131c9ca4d5624e74c3f8273ccc926f9b",
)

load("//tools/workspace:pypi.bzl", "pypi_archive")

pypi_archive(
    name = "six_archive",
    package = "six",
    version = "1.10.0",
    sha256 = "105f8d68616f8248e24bf0e9372ef04d3cc10104f1980f54d57b2ce73a5ad56a",  # noqa
    build_file = "@drake//tools/workspace/six:package.BUILD.bazel",
)

bind(
    name = "six",
    actual = "@six_archive//:six",
)

pypi_archive(
    name = "semantic_version",
    version = "2.6.0",
    sha256 = "2a4328680073e9b243667b201119772aefc5fc63ae32398d6afafff07c4f54c0",  # noqa
    strip_prefix = "semantic_version",
    build_file = "@drake//tools/workspace/semantic_version:package.BUILD.bazel",  # noqa
)

github_archive(
    name = "pycps",
    repository = "mwoehlke/pycps",
    commit = "544c1ded81b926a05b3dedb06504bd17bc8d0a95",
    sha256 = "0b97cbaae107e5ddbe89073b6e42b679130f1eb81b913aa93da9e72e032a137b",  # noqa
    build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
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
    commit = "392237e10ba4",
    sha256 = "44068bb91c07c9305213057cad801ae5b689ac1a5f37cd8330dd6e729df8f5b0",  # noqa
    strip_prefix = "ignitionrobotics-ign-math-392237e10ba4",
    build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",
)

bitbucket_archive(
    name = "ignition_rndf",
    repository = "ignitionrobotics/ign-rndf",
    commit = "ignition-rndf_0.1.5",
    sha256 = "fa1033be146ff51f3b2c679ff160838c1e3ca736c565b19510a5c9b6d352fbaf",  # noqa
    strip_prefix = "ignitionrobotics-ign-rndf-214a333fbdcb",
    build_file = "@drake//tools/workspace/ignition_rndf:package.BUILD.bazel",
)

load("//tools/workspace/boost:package.bzl", "boost_repository")

boost_repository(name = "boost")

bitbucket_archive(
    name = "sdformat",
    repository = "osrf/sdformat",
    commit = "bac3dfb42cc7",
    sha256 = "212211eddd9fa010b4b61a2dae87cd84a66a8b78ed302612d214b7388f9bc198",  # noqa
    strip_prefix = "osrf-sdformat-bac3dfb42cc7",
    build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
)

load("//tools/workspace/vtk:package.bzl", "vtk_repository")

vtk_repository(name = "vtk")

load("//tools/workspace/expat:package.bzl", "expat_repository")

expat_repository(name = "expat")

load("//tools/workspace/glew:package.bzl", "glew_repository")

glew_repository(name = "glew")

load("//tools/workspace/liblz4:package.bzl", "liblz4_repository")

liblz4_repository(name = "liblz4")

load("//tools/workspace/libpng:package.bzl", "libpng_repository")

libpng_repository(name = "libpng")

load("//tools/workspace/tinyxml:package.bzl", "tinyxml_repository")

tinyxml_repository(name = "tinyxml")

load("//tools/workspace/tinyxml2:package.bzl", "tinyxml2_repository")

tinyxml2_repository(name = "tinyxml2")

load("//tools/workspace/zlib:package.bzl", "zlib_repository")

zlib_repository(name = "zlib")

load("//tools/workspace/drake_visualizer:package.bzl", "drake_visualizer_repository")  # noqa

drake_visualizer_repository(name = "drake_visualizer")
