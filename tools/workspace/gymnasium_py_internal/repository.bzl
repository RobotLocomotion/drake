load("//tools/workspace:github.bzl", "github_archive")

def gymnasium_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        upgrade_type = "release",
        exclude_tags_pattern = "v[0-9.]+a[0-9]+",
        commit = "v1.3.0",
        sha256 = "600df166dd6aca13281130a80eff1a23f4f867a5c919ed614743806e7e5efeaa",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
