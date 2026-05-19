load("//tools/workspace:github.bzl", "github_archive")

def nanobind_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "wjakob/nanobind",
        commit = "v2.12.0",
        sha256 = "01f1f0cd0398743c18f33d07ae36ad410bd7f4a1e90683b508504de897d6e629",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/ndarray_extra_import.patch",
            ":patches/ndarray_opaque_handle.patch",
        ],
        mirrors = mirrors,
    )
