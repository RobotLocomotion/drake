load("//tools/workspace:github.bzl", "github_archive")

def nlopt_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "stevengj/nlopt",
        commit = "v2.8.0",
        sha256 = "e02a4956a69d323775d79fdaec7ba7a23ed912c7d45e439bc933d991ea3193fd",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/remove_luksan.patch",
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
