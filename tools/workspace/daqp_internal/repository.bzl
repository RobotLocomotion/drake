load("//tools/workspace:github.bzl", "github_archive")

def daqp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "darnstrom/daqp",
        upgrade_type = "release",
        commit = "v0.10.3",
        sha256 = "623fd351adcccb3d491fb800b443146d418e1f1af94d34a72f29f6eb78f4ae94",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
