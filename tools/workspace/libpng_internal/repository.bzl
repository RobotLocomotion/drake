load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.44",
        sha256 = "0ef5b633d0c65f780c4fced27ff832998e71478c13b45dfb6e94f23a82f64f7c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
