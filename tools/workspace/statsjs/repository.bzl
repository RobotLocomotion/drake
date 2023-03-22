load("@drake//tools/workspace:github.bzl", "github_archive")

def statsjs_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mrdoob/stats.js",
        commit = "1ecb62cd10f30789b540dcdbbd473f1de6eed614",
        sha256 = "47107bf21c6369b7ed6d88ed827f03f408c3f85a4686f01d01e80f3fde87184c",  # noqa
        build_file = "@drake//tools/workspace/statsjs:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
