# -*- python -*-

load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/workspace:github.bzl", "github_archive")

# Using the `drake` branch of this repository.
_REPOSITORY = "RobotLocomotion/pybind11"

# When upgrading this commit, check the version header within
#  https://github.com/RobotLocomotion/pybind11/blob/drake/include/pybind11/detail/common.h
# and if it has changed, then update the version number in the two
# pybind11-*.cmake files in the current directory to match.
_COMMIT = "25eac331ce222f7cd6cca5f3e023daa0ab1cb475"

_SHA256 = "2736e9407efb1243c76f32261fdb29625657ff53f74469505e47d6340dc60a9f"

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
