load("//tools/workspace:github.bzl", "github_archive")

def cxxfilt_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "afq984/python-cxxfilt",
        commit = "v0.3.0",
        sha256 = "117ecbf4ce2b3c0dcbbe200824783f8547b418930a5e599bbb369bb67138bd97",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
