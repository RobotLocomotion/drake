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

load("//tools:bitbucket.bzl", "bitbucket_archive")
load("//tools:github.bzl", "github_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

local_repository(
    name = "kythe",
    path = "tools/third_party/kythe",
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
    sha256 = "58a6f4277ca2bc8565222b3bbd58a177609e9c488e8a72649359ba51450db7d8",  # noqa
    build_file = "tools/gtest.BUILD",
)

# When updating the version of gflags, update tools/install/gflags/gflags.cps
github_archive(
    name = "com_github_gflags_gflags",
    repository = "gflags/gflags",
    commit = "95ffb27c9c7496ede1409e042571054c70cb9519",
    sha256 = "723c21f783c720c0403c9b44bf500d1961a08bd2635cbc117107af22d2e1643f",  # noqa
)

github_archive(
    name = "google_styleguide",
    repository = "google/styleguide",
    commit = "159b4c81bbca97a9ca00f1195a37174388398a67",
    sha256 = "3ed86946e6e637f0fe21749c0323b086e62c4b8b93694d6cedad615cdc584512",  # noqa
    build_file = "tools/google_styleguide.BUILD",
)

github_archive(
    name = "pycodestyle",
    repository = "PyCQA/pycodestyle",
    commit = "2.3.1",
    sha256 = "e9fc1ca3fd85648f45c0d2e33591b608a17d8b9b78e22c5f898e831351bacb03",  # noqa
    build_file = "tools/pycodestyle.BUILD",
)

bitbucket_archive(
    name = "eigen",
    repository = "eigen/eigen",
    # N.B. See #5785; do your best not to have to bump this to a newer commit.
    commit = "3.3.3",
    sha256 = "94878cbfa27b0d0fbc64c00d4aafa137f678d5315ae62ba4aecddbd4269ae75f",  # noqa
    strip_prefix = "eigen-eigen-67e894c6cd8f",
    build_file = "tools/eigen.BUILD",
)

github_archive(
    name = "spdlog",
    repository = "gabime/spdlog",
    commit = "v0.13.0",
    sha256 = "d798a6ca19165f0a18a43938859359269f5a07fd8e0eb83ab8674739c9e8f361",  # noqa
    build_file = "tools/spdlog.BUILD",
)

github_archive(
    name = "fmt",
    repository = "fmtlib/fmt",
    commit = "3.0.1",
    sha256 = "dce62ab75a161dd4353a98364feb166d35e7eea382169d59d9ce842c49c55bad",  # noqa
    build_file = "tools/fmt.BUILD",
)

# In the unlikely event that you update the version here, verify that the
# licenses in tools/third_party/jchart2d/LICENSE are still applicable.
maven_jar(
    name = "net_sf_jchart2d_jchart2d",
    artifact = "net.sf.jchart2d:jchart2d:3.3.2",
    sha1 = "4950821eefe4c204903e68b4d45a558b5ebdd6fa",
)

maven_jar(
    name = "com_jidesoft_jide_oss",
    artifact = "com.jidesoft:jide-oss:2.9.7",
    sha1 = "a9bb0d8384012c25c1519f6dd9adc80dd720a050",
)

maven_jar(
    name = "commons_io_commons_io",
    artifact = "commons-io:commons-io:1.3.1",
    sha1 = "b90b6ac57cf27a2858eaa490d02ba7945d18ca7b",
)

maven_jar(
    name = "org_apache_xmlgraphics_xmlgraphics_commons",
    artifact = "org.apache.xmlgraphics:xmlgraphics-commons:1.3.1",
    sha1 = "f7d0fa54e2750acd82b1a241c043be6fce1bf0dc",
)

github_archive(
    name = "lcm",
    repository = "lcm-proj/lcm",
    commit = "c0a0093a950fc83e12e8d5918a0319b590356e7e",
    sha256 = "d5bb1a0153b9c1526590e7d65be8ca79e4f5e9bf4ce58178c992eaca49d17fb0",  # noqa
    build_file = "tools/lcm.BUILD",
)

# In the unlikely event that you update the version here, verify that the
# licenses in tools/third_party/libbot/ldpc LICENSE are still applicable.
github_archive(
    name = "libbot",
    repository = "RobotLocomotion/libbot2",
    commit = "495ae366d5e380b58254368217fc5c798e72aadd",
    sha256 = "c463460a4dd6133d6d21e6ab6e493fdcdca442d2df86bcb56749f6740bc61db5",  # noqa
    build_file = "tools/libbot.BUILD",
)

github_archive(
    name = "bullet",
    repository = "bulletphysics/bullet3",
    commit = "2.86.1",
    sha256 = "c058b2e4321ba6adaa656976c1a138c07b18fc03b29f5b82880d5d8228fbf059",  # noqa
    build_file = "tools/bullet.BUILD",
)

github_archive(
    name = "ccd",
    repository = "danfis/libccd",
    commit = "v2.0",
    sha256 = "1b4997e361c79262cf1fe5e1a3bf0789c9447d60b8ae2c1f945693ad574f9471",  # noqa
    build_file = "tools/ccd.BUILD",
)

github_archive(
    name = "octomap",
    repository = "OctoMap/octomap",
    commit = "v1.7.2",
    sha256 = "fe55efbb9ebf2b3388860e54b1c8a53d23e5a05de5956c043278013e01066c34",  # noqa
    build_file = "tools/octomap.BUILD",
)

github_archive(
    name = "fcl",
    repository = "flexible-collision-library/fcl",
    commit = "06d48b3b6f3605b8caf119d5208d8156eb64fe0d",
    sha256 = "0a5652cac609cca58f85d68c08298e177867188ad730e78c8c9ac97eea8d32b4",  # noqa
    build_file = "tools/fcl.BUILD",
)

github_archive(
    name = "ipopt",
    repository = "RobotLocomotion/ipopt-mirror",
    commit = "aecf5abd3913eebf1b99167c0edd4e65a6b414bc",
    sha256 = "4ddde882913b9edc91f281edcdffccdd5343a8b6f1bc42b541188f49159e9768",  # noqa
    build_file = "tools/ipopt.BUILD",
)

github_archive(
    name = "nlopt",
    repository = "stevengj/nlopt",
    commit = "516aca7e96405939726648e00faeb26bd2c9b29f",
    sha256 = "6041ca30072b354ed3c235743779bf17dacf6199b2b30746c499f65082665d5f",  # noqa
    build_file = "tools/nlopt.BUILD",
)

github_archive(
    name = "optitrack_driver",
    repository = "RobotLocomotion/optitrack-driver",
    commit = "b9a59b66cb0627f9f174e11f323fdcf6cb223bb6",
    sha256 = "5c9d917fcb9d325ceba75484a2d3f31ea044a090a966ac1ee2c4afd91923039e",  # noqa
)

github_archive(
    name = "pybind11",
    repository = "RobotLocomotion/pybind11",
    commit = "6d72785766558047ee2e2075198c07d8c25eb631",
    sha256 = "08b4813b3b17f607efc4e8ba8b73bf55759ba744cab125e9fc666b5161cb1d0a",  # noqa
    build_file = "tools/pybind11.BUILD",
)

github_archive(
    name = "lcmtypes_bot2_core",
    repository = "openhumanoids/bot_core_lcmtypes",
    commit = "99676541398749c2aab4b5b2c38be77d268085cc",
    sha256 = "896fd3edf87c7dfaae378af12d52d233577cc495ae96b5076c48b5b9ca700b4a",  # noqa
    build_file = "tools/lcmtypes_bot2_core.BUILD",
)

github_archive(
    name = "lcmtypes_robotlocomotion",
    repository = "RobotLocomotion/lcmtypes",
    commit = "8aea7a94d53dea01bfceba5f3cbe8e8cc9fb0244",
    sha256 = "f23a143d7865ea4f6cd9aeb2211fe36e20712a39d439cf16fea2b11685f29b61",  # noqa
    build_file = "tools/lcmtypes_robotlocomotion.BUILD",
)

github_archive(
    name = "tinyobjloader",
    repository = "syoyo/tinyobjloader",
    commit = "v1.0.6",
    sha256 = "19ee82cd201761954dd833de551edb570e33b320d6027e0d91455faf7cd4c341",  # noqa
    build_file = "tools/tinyobjloader.BUILD",
)

# Necessary for buildifier.
github_archive(
    name = "io_bazel_rules_go",
    repository = "bazelbuild/rules_go",
    commit = "0.4.4",
    sha256 = "afec53d875013de6cebe0e51943345c587b41263fdff36df5ff651fbf03c1c08",  # noqa
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
    # TODO(mwoehlke-kitware): Bump this commit to a release tag once it is
    # incorporated in a released version.
    commit = "7ce605fb1585076ed681e37d82d0ef529244b23a",
    sha256 = "c6210992d328212a7752a2c888a15f5c597dbf31f03ac0d59457ceff2928a30b",  # noqa
)

github_archive(
    name = "yaml_cpp",
    repository = "jbeder/yaml-cpp",
    commit = "85af926ddc5f3c8fb438001743e65ec3a039ceec",
    sha256 = "d94cdb84f346ce4d9f1f891505ed257796103f70ce56590bdd02e025c8503b16",  # noqa
    build_file = "tools/yaml_cpp.BUILD",
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
    sha256 = "105f8d68616f8248e24bf0e9372ef04d3cc10104f1980f54d57b2ce73a5ad56a",  # noqa
    build_file = "tools/six.BUILD",
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
    sha256 = "0a0ae63cbffc274efb573bdde9a253e3f32e458c41261df51c5dbc5ad541e8f7",  # noqa
)

pypi_archive(
    name = "semantic_version",
    version = "2.6.0",
    sha256 = "2a4328680073e9b243667b201119772aefc5fc63ae32398d6afafff07c4f54c0",  # noqa
    strip_prefix = "semantic_version",
    build_file = "tools/semantic_version.BUILD",
)

github_archive(
    name = "pycps",
    repository = "mwoehlke/pycps",
    commit = "adff2def458928902ad482337330676beeeedb93",
    sha256 = "61fd6f1810724c50784da97ef666c4b5b9110a8ce57f79b4c5510d6f8bb7c75e",  # noqa
    build_file = "tools/pycps.BUILD",
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
    build_file = "tools/ignition_math.BUILD",
)

bitbucket_archive(
    name = "ignition_rndf",
    repository = "ignitionrobotics/ign-rndf",
    commit = "ignition-rndf_0.1.5",
    sha256 = "fa1033be146ff51f3b2c679ff160838c1e3ca736c565b19510a5c9b6d352fbaf",  # noqa
    strip_prefix = "ignitionrobotics-ign-rndf-214a333fbdcb",
    build_file = "tools/ignition_rndf.BUILD",
)

load("//tools:boost.bzl", "boost_repository")

boost_repository(
    name = "boost",
)

bitbucket_archive(
    name = "sdformat",
    repository = "osrf/sdformat",
    commit = "bac3dfb42cc7",
    sha256 = "b10a3ac68ed46f8d5780ddc687e6c89c71cb4c1e4e65449197f8aac76be903d8",  # noqa
    strip_prefix = "osrf-sdformat-bac3dfb42cc7",
    build_file = "tools/sdformat.BUILD",
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
    name = "tinyxml",
    modname = "tinyxml",
)

pkg_config_package(
    name = "zlib",
    modname = "zlib",
)

load("//tools:director.bzl", "director_repository")

director_repository(
    name = "director",
)
