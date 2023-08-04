load("@drake//tools/workspace:github.bzl", "github_archive")

def gtest_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/googletest",
        commit = "v1.14.0",
        sha256 = "8ad598c73ad796e0d8280b082cebd82a630d73e73cd3c70057938a6501bba5d7",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/add_printers.patch",
        ],
        mirrors = mirrors,
    )
