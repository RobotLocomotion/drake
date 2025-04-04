load("//tools/workspace:github.bzl", "github_archive")

def cpplint_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cpplint/cpplint",
        commit = "2.0.1",
        sha256 = "49cffdaad85a19a760811f8ce7736293f994d8eabee7c11fcf8bb72075a84239",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/transitive_includes.patch",
            ":patches/whitespace_newline.patch",
        ],
        mirrors = mirrors,
    )
