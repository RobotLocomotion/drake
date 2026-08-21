load("//tools/workspace:github.bzl", "github_archive")

def nanoflann_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "jlblancoc/nanoflann",
        commit = "v1.9.0",
        sha256 = "14dc863ec47d52ec3272b4fd409fd198a52e6cab58ece70b1da9c3dc2e478942",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/namespace.patch",
        ],
        mirrors = mirrors,
    )
