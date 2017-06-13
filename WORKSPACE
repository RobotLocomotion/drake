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
#     sha256 = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
# )

workspace(name = "drake")

load("//tools:bitbucket.bzl", "bitbucket_archive")
load("//tools:github.bzl", "github_archive")
load('@bazel_tools//tools/build_defs/repo:git.bzl', 'git_repository')

local_repository(
    name = "kythe",
    path = "tools/third_party/kythe",
)
load("@kythe//tools/build_rules/config:pkg_config.bzl", "pkg_config_package")

pkg_config_package(
    name = "glib",
    modname = "glib-2.0",
)

load("//tools:python.bzl", "python_repository")
python_repository(
    name = "python",
    version = "2.7",
)

load("//tools:numpy.bzl", "numpy_repository")
numpy_repository(
    name = "numpy",
    python_version = "2.7",
)

github_archive(
    name = "gtest",
    repository = "google/googletest",
    commit = "release-1.8.0",
    sha256 = "58a6f4277ca2bc8565222b3bbd58a177609e9c488e8a72649359ba51450db7d8",
    build_file = "tools/gtest.BUILD",
)

github_archive(
    name = "com_github_gflags_gflags",
    repository = "gflags/gflags",
    commit = "95ffb27c9c7496ede1409e042571054c70cb9519",
    sha256 = "723c21f783c720c0403c9b44bf500d1961a08bd2635cbc117107af22d2e1643f",
)

github_archive(
    name = "google_styleguide",
    repository = "google/styleguide",
    commit = "159b4c81bbca97a9ca00f1195a37174388398a67",
    sha256 = "3ed86946e6e637f0fe21749c0323b086e62c4b8b93694d6cedad615cdc584512",
    build_file = "tools/google_styleguide.BUILD",
)

github_archive(
    name = "pycodestyle",
    repository = "PyCQA/pycodestyle",
    commit = "2.3.1",
    sha256 = "e9fc1ca3fd85648f45c0d2e33591b608a17d8b9b78e22c5f898e831351bacb03",
    build_file = "tools/pycodestyle.BUILD",
)

bitbucket_archive(
    name = "eigen",
    repository = "eigen/eigen",
    # N.B. See #5785; do your best not to have to bump this to a newer commit.
    commit = "3.3.3",
    sha256 = "94878cbfa27b0d0fbc64c00d4aafa137f678d5315ae62ba4aecddbd4269ae75f",
    build_file = "tools/eigen.BUILD",
    strip_prefix = "eigen-eigen-67e894c6cd8f",
)

github_archive(
    name = "spdlog",
    repository = "gabime/spdlog",
    commit = "43a4048b92ef5b7eff6dc637a621c7da3a41d194",
    build_file = "tools/spdlog.BUILD",
    sha256 = "5166c036eacd625b86f725bfba356547e0bc497232649662c61cde7b1b423292",
)

github_archive(
    name = "fmt",
    repository = "fmtlib/fmt",
    commit = "3.0.1",
    build_file = "tools/fmt.BUILD",
    sha256 = "dce62ab75a161dd4353a98364feb166d35e7eea382169d59d9ce842c49c55bad",
)

# In the unlikely event that you update the version here, verify that the
# licenses in tools/third_party/jchart2d/LICENSE are still applicable.
maven_jar(
    name = "net_sf_jchart2d_jchart2d",
    artifact = "net.sf.jchart2d:jchart2d:3.3.2",
    sha1 = "4950821eefe4c204903e68b4d45a558b5ebdd6fa",
)

github_archive(
    name = "lcm",
    repository = "lcm-proj/lcm",
    commit = "c0a0093a950fc83e12e8d5918a0319b590356e7e",
    build_file = "tools/lcm.BUILD",
    sha256 = "d5bb1a0153b9c1526590e7d65be8ca79e4f5e9bf4ce58178c992eaca49d17fb0",
)

github_archive(
    name = "libbot",
    repository = "RobotLocomotion/libbot2",
    commit = "495ae366d5e380b58254368217fc5c798e72aadd",
    build_file = "tools/libbot.BUILD",
    sha256 = "c463460a4dd6133d6d21e6ab6e493fdcdca442d2df86bcb56749f6740bc61db5",
)

github_archive(
    name = "bullet",
    repository = "RobotLocomotion/bullet3",
    commit = "ae2c4ca0618d55c6a29900aed75b958604149fdb",
    build_file = "tools/bullet.BUILD",
    sha256 = "2121dc8b0d33f50adbad8fc9ac5e007141df5cf5738fce72200c5bac4ffc589b",
)

github_archive(
    name = "ccd",
    repository = "danfis/libccd",
    commit = "16b9379fb6e8610566fe5e1396166daf7106f165",
    build_file = "tools/ccd.BUILD",
    sha256 = "fc583888c731d91c9ef287ca9ba443426ecfce75056d6c468b063b383bd8efa1",
)

github_archive(
    name = "octomap",
    repository = "OctoMap/octomap",
    commit = "6d7c31ae4df2c93cb8a954e44d442338b58d3558",
    build_file = "tools/octomap.BUILD",
    sha256 = "023ddd5b1e8ed1a70374c352cbd5b02bd5a26707f61a13cfb087766d5ca466e3",
)

github_archive(
    name = "fcl",
    repository = "flexible-collision-library/fcl",
    commit = "06d48b3b6f3605b8caf119d5208d8156eb64fe0d",
    build_file = "tools/fcl.BUILD",
    sha256 = "0a5652cac609cca58f85d68c08298e177867188ad730e78c8c9ac97eea8d32b4",
)

github_archive(
    name = "ipopt",
    repository = "RobotLocomotion/ipopt-mirror",
    commit = "aecf5abd3913eebf1b99167c0edd4e65a6b414bc",
    build_file = "tools/ipopt.BUILD",
    sha256 = "4ddde882913b9edc91f281edcdffccdd5343a8b6f1bc42b541188f49159e9768",
)

github_archive(
    name = "nlopt",
    repository = "stevengj/nlopt",
    commit = "516aca7e96405939726648e00faeb26bd2c9b29f",
    build_file = "tools/nlopt.BUILD",
    sha256 = "6041ca30072b354ed3c235743779bf17dacf6199b2b30746c499f65082665d5f",
)

github_archive(
    name = "optitrack_driver",
    repository = "RobotLocomotion/optitrack-driver",
    commit = "3a5da8d7c66c95ca98cda4dc7ca604f681464168",
    sha256 = "a4d4c61ed5af59f12a273629eb28fa95ac2349abffe8912468bc5cf6dff34d28",
)

github_archive(
    name = "pybind11",
    repository = "RobotLocomotion/pybind11",
    commit = "6d72785766558047ee2e2075198c07d8c25eb631",
    build_file = "tools/pybind11.BUILD",
    sha256 = "08b4813b3b17f607efc4e8ba8b73bf55759ba744cab125e9fc666b5161cb1d0a",
)

github_archive(
    name = "bot_core_lcmtypes",
    repository = "openhumanoids/bot_core_lcmtypes",
    commit = "99676541398749c2aab4b5b2c38be77d268085cc",
    build_file = "tools/bot_core_lcmtypes.BUILD",
    sha256 = "896fd3edf87c7dfaae378af12d52d233577cc495ae96b5076c48b5b9ca700b4a",
)

github_archive(
    name = "robotlocomotion_lcmtypes",
    repository = "RobotLocomotion/lcmtypes",
    commit = "8aea7a94d53dea01bfceba5f3cbe8e8cc9fb0244",
    build_file = "tools/robotlocomotion_lcmtypes.BUILD",
    sha256 = "f23a143d7865ea4f6cd9aeb2211fe36e20712a39d439cf16fea2b11685f29b61",
)

github_archive(
    name = "tinyobjloader",
    repository = "syoyo/tinyobjloader",
    commit = "9d9e987c4776d9df54e0ab65639e1befddb1d5ae",
    build_file = "tools/tinyobjloader.BUILD",
    sha256 = "e5c7ce01a153a3d9101f6c197e10145490ecd84ba7d0b6463708a60bc2845d4b",
)

# Necessary for buildifier.
github_archive(
    name = "io_bazel_rules_go",
    repository = "bazelbuild/rules_go",
    commit = "0.4.4",
    sha256 = "afec53d875013de6cebe0e51943345c587b41263fdff36df5ff651fbf03c1c08",
)

# Necessary for buildifier.
load("@io_bazel_rules_go//go:def.bzl", "go_repositories", "new_go_repository")

# Necessary for buildifier.
go_repositories()

# Necessary for buildifier.
new_go_repository(
    name = "org_golang_x_tools",
    commit = "3d92dd60033c312e3ae7cac319c792271cf67e37",
    importpath = "golang.org/x/tools",
)

github_archive(
    name = "com_github_bazelbuild_buildtools",
    repository = "bazelbuild/buildtools",
    commit = "0.4.5",
    sha256 = "7a732ea12d88ddbf9adc99ff5b5c39bfda53b6286ecc79c3bc082d5f53f46f44",
)

github_archive(
    name = "yaml_cpp",
    repository = "jbeder/yaml-cpp",
    commit = "85af926ddc5f3c8fb438001743e65ec3a039ceec",
    build_file = "tools/yaml_cpp.BUILD",
    sha256 = "d94cdb84f346ce4d9f1f891505ed257796103f70ce56590bdd02e025c8503b16",
)

load("//tools:gurobi.bzl", "gurobi_repository")
gurobi_repository(
    name = "gurobi",
)

load("//tools:mosek.bzl", "mosek_repository")
mosek_repository(
    name = "mosek",
)

load("//tools:gfortran.bzl", "gfortran_repository")
gfortran_repository(
    name = "gfortran",
)

git_repository(
  name = "snopt",
  remote = "git@github.com:RobotLocomotion/snopt.git",
  commit = "2ec980370eeb72897135b11570033a19bda885a7",
)

# Python Libraries
load("//tools:pypi.bzl", "pypi_archive")
pypi_archive(
    name = "six_archive",
    package = "six",
    version = "1.10.0",
    sha256 = "105f8d68616f8248e24bf0e9372ef04d3cc10104f1980f54d57b2ce73a5ad56a",
    build_file = "tools/six.BUILD",
)

bind(
    name = "six",
    actual = "@six_archive//:six",
)

github_archive(
    name = "protobuf",
    repository = "google/protobuf",
    commit = "v3.1.0",
    sha256 = "0a0ae63cbffc274efb573bdde9a253e3f32e458c41261df51c5dbc5ad541e8f7",
)

pypi_archive(
    name = "semantic_version",
    version = "2.6.0",
    sha256 = "2a4328680073e9b243667b201119772aefc5fc63ae32398d6afafff07c4f54c0",
    build_file = "tools/semantic_version.BUILD",
    strip_prefix = "semantic_version",
)

github_archive(
    name = "pycps",
    repository = "mwoehlke/pycps",
    commit = "d68a10ce1130f87d38a13ae42ddb263042e2352a",
    sha256 = "4de60f6b260b286dc2e68e9cdc31decc8f9ef43f77894c3d33a6fd097549008b",
    build_file = "tools/pycps.BUILD",
)

# The "@python_headers//:python_headers" target is required by protobuf
# during "bazel query" but not "bazel build", so a stub is fine.
new_local_repository(
    name = "python_headers",
    path = "not/real/stub",
    build_file_content = ("cc_library(name = 'python_headers', " +
                          "visibility = ['//visibility:public'])")
)

bitbucket_archive(
    name = "ignition_math",
    repository = "ignitionrobotics/ign-math",
    commit = "ignition-math3_3.2.0",
    sha256 = "1948c1610fa4403bce7ba2a262a29662990ee66aab00882411a0868afe0e5309",
    build_file = "tools/ignition_math.BUILD",
    strip_prefix = "ignitionrobotics-ign-math-e86e5bb392e4"
)

bitbucket_archive(
    name = "ignition_rndf",
    repository = "ignitionrobotics/ign-rndf",
    commit = "ignition-rndf_0.1.5",
    sha256 = "fa1033be146ff51f3b2c679ff160838c1e3ca736c565b19510a5c9b6d352fbaf",
    build_file = "tools/ignition_rndf.BUILD",
    strip_prefix = "ignitionrobotics-ign-rndf-214a333fbdcb",
)

bitbucket_archive(
    name = "sdformat",
    repository = "osrf/sdformat",
    commit = "deca28cd6cd5",
    sha256 = "d89a03178ef71d0a222247bf3fc4ccb8c490aebe83516f7290181d64e5da8dac",
    build_file = "tools/sdformat.BUILD",
    strip_prefix = "osrf-sdformat-deca28cd6cd5",
)

load("//tools:vtk.bzl", "vtk_repository")
vtk_repository(
    name = "vtk",
)

pkg_config_package(
    name = "libpng",
    modname = "libpng",
)

pkg_config_package(
    name = "zlib",
    modname = "zlib",
)

load("//tools:director.bzl", "director_repository")
director_repository(
    name = "director",
)
