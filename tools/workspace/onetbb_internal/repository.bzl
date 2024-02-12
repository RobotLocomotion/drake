load("//tools/workspace:github.bzl", "github_archive")

def onetbb_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oneapi-src/oneTBB",
        commit = "v2021.8.0",
        # TODO(jwnimmer-tri) We are using the TBB headers from this repository,
        # but the TBB library from MOSEK's binary release. We'll probably need
        # some tooling to keep all of that in sync, but for now we'll just pin.
        commit_pin = 1,
        sha256 = "eee380323bb7ce864355ed9431f85c43955faaae9e9bce35c62b372d7ffd9f8b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
