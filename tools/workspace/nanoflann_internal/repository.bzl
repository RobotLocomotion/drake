load("//tools/workspace:github.bzl", "github_archive")

def nanoflann_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "jlblancoc/nanoflann",
        commit = "v1.6.1",
        sha256 = "e258d6fd6ad18e1809fa9c081553e78036fd6e7b418d3762875c2c5a015dd431",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/namespace.patch",
        ],
        mirrors = mirrors,
    )
