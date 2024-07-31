load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.7.0",
        sha256 = "529b067f5d80981f45ddf6766627b8fc5af619822f068f342aab776e683df4f3",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
