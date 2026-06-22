load("//tools/workspace:github.bzl", "github_archive")

def nlopt_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "stevengj/nlopt",
        upgrade_type = "release",
        commit = "v2.11.0",
        sha256 = "53e552d83e9294d67db37f0f4a23f15933a9ef698485301a18b98b40004cf0de",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
