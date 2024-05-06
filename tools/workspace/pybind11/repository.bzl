load("//tools/workspace:generate_file.bzl", "generate_file")
load("//tools/workspace:github.bzl", "github_archive")

_REPOSITORY = "pybind/pybind11"

# When upgrading this commit, also fix the version number in the two
# pybind11-*.cmake files in the current directory to match.
_COMMIT = "v2.11.1"

_SHA256 = "d475978da0cdc2d43b73f30910786759d593a9d8ee05b1b6846d1eb16c6d2e0c"

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
        content = '''# noqa: shebang
"""
Provides information on the external fork of `pybind11` used by `pydrake`.
"""

repository = {repository}
commit = {commit}
sha256 = {sha256}
'''.format(**vars),
    )
