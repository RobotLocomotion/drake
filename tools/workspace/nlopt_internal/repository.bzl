load("//tools/workspace:github.bzl", "github_archive")

def nlopt_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "stevengj/nlopt",
        commit = "v2.9.1",
        sha256 = "1e6c33f8cbdc4138d525f3326c231f14ed50d99345561e85285638c49b64ee93",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
