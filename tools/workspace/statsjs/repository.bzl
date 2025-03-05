load("//tools/workspace:github.bzl", "github_archive")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

def statsjs_repository(
        name,
        mirrors = None,
        _is_drake_self_call = False):
    if not _is_drake_self_call:
        print_warning("statsjs_repository")
    github_archive(
        name = name,
        repository = "mrdoob/stats.js",
        commit = "71e88f65280fd5c6e91d1f84f0f633d372ed7eae",
        sha256 = "ae93155dba921f53174af038a4f5e2f4f1a4e8107b1996d58a5a50cf55234e8e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
