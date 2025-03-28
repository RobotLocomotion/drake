load("//tools/workspace:generate_file.bzl", "generate_file")
load("//tools/workspace:github.bzl", "github_archive")

# Any time you change either the _REPOSITORY or _COMMIT below, check the
# version header within the file `include/pybind11/detail/common.h` in the
# branch indicated by `_REPOSITORY` and `_COMMIT`, and if it has changed, then
# update the version number in the two pybind11-*.cmake files in the current
# directory to match.
_REPOSITORY = "pybind/pybind11"
_COMMIT = "v2.13.6"

_SHA256 = "e08cb87f4773da97fa7b5f035de8763abc656d87d5773e62f6da0587d1f0ec20"

def pybind11_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = _REPOSITORY,
        commit = _COMMIT,
        sha256 = _SHA256,
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/check_signature_infection.patch",
            ":patches/eigen_object_matrices.patch",
            ":patches/shared_ptr_lifetime.patch",
        ],
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
