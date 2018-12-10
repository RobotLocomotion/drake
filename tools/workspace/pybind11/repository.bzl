# -*- python -*-

load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/workspace:github.bzl", "github_archive")

_REPOSITORY = "RobotLocomotion/pybind11"

_COMMIT = "5501496d6984d81bef3a0dacd9a2e368d544bfce"

_SHA256 = "03f880238bdd4c29536448eb3999a75d38ff677ed1b3e38cdca0e8fd35753262"

def pybind11_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = _REPOSITORY,
        commit = _COMMIT,
        sha256 = _SHA256,
        build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
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
