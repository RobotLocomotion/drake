load("//tools/workspace:github.bzl", "github_archive")

def nlopt_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "stevengj/nlopt",
        commit = "v2.10.1",
        sha256 = "30d13ce16da119db3e987784f7864e35a562ec62c186352fae55cd003e6c58ff",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
