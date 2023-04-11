load("@drake//tools/workspace:github.bzl", "github_archive")

def statsjs_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mrdoob/stats.js",
        commit = "b235d9c1e9c90c95b59d69bba53565c65bb2f694",
        sha256 = "3ae5eaa7f8a2a52296a6f13e9b423a8e7bdf7354ae58f056b966c7359c3dbd4e",  # noqa
        build_file = "@drake//tools/workspace/statsjs:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
