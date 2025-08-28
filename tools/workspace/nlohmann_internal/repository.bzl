load("//tools/workspace:github.bzl", "github_archive")

def nlohmann_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nlohmann/json",
        commit = "v3.12.0",
        sha256 = "4b92eb0c06d10683f7447ce9406cb97cd4b453be18d7279320f7b2f025c10187",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
