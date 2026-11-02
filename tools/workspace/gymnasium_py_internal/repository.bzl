load("//tools/workspace:github.bzl", "github_archive")

def gymnasium_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        upgrade_type = "release",
        exclude_tags_pattern = "v[0-9.]+a[0-9]+",
        commit = "v1.4.0",
        sha256 = "c34c99e18d4121aff526b0768e50b30acbe23d05c7285fc681655406d527359c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
