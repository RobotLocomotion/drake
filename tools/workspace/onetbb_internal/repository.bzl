load("//tools/workspace:github.bzl", "github_archive")

def onetbb_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oneapi-src/oneTBB",
        # TODO(jwnimmer-tri) Figure out how to sync this with MOSEK's version.
        commit = "v2021.8.0",
        sha256 = "eee380323bb7ce864355ed9431f85c43955faaae9e9bce35c62b372d7ffd9f8b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
