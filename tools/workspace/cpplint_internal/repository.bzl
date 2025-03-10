load("//tools/workspace:github.bzl", "github_archive")

def cpplint_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cpplint/cpplint",
        commit = "80da3c1ef37af017715dc1366fbda8263179bdeb",
        sha256 = "006d8ce87a2dcc8a644b78fe0c2d4d45bc52ca41414ea66d3c7dcd6ae3e61d9e",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/transitive_includes.patch",
            ":patches/whitespace_newline.patch",
        ],
        mirrors = mirrors,
    )
