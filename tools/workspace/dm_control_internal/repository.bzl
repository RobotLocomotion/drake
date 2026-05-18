load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.40",
        sha256 = "0019de4e870b02ec20f293ed85afb651352194c6e50dd2ee99137c75dc474d17",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
