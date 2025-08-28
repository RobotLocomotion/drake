load("//tools/workspace:github.bzl", "github_archive")

def cpplint_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cpplint/cpplint",
        commit = "2.0.2",
        sha256 = "fc6d0cd40f934b58e8e0bb5eb5f1f2b651880b5fbc0e93a54e6fb6503f733b3d",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/transitive_includes.patch",
        ],
        mirrors = mirrors,
    )
