# -*- python -*-

load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/workspace:github.bzl", "github_archive")

# Using the `drake` branch of this repository.
_REPOSITORY = "RobotLocomotion/pybind11"

# When upgrading this commit, check the version header within
#  https://github.com/RobotLocomotion/pybind11/blob/drake/include/pybind11/detail/common.h
# and if it has changed, then update the version number in the two
# pybind11-*.cmake files in the current directory to match.
_COMMIT = "36695db542e1c34f6db7ca3ebb0415de30762395"

_SHA256 = "36b35032ae6279af6257588856b0284b8106221c816b26509a05801c54d60e30"

def pybind11_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = _REPOSITORY,
        commit = _COMMIT,
        sha256 = _SHA256,
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )

def generate_pybind11_version_py_file(name):
    vars = dict(
        repository = repr(_REPOSITORY),
        commit = repr(_COMMIT),
        sha256 = repr(_SHA256),
    )
    generate_file(
        name = name,
        content = '''
"""
Provides information on the external fork of `pybind11` used by `pydrake`.
"""

repository = {repository}
commit = {commit}
sha256 = {sha256}
'''.format(**vars),
    )
