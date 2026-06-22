load("//tools/workspace:github.bzl", "github_archive")

def github3_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sigmavirus24/github3.py",
        upgrade_type = "tag",
        # github3.py has some older tags beginning with "v" (plus some other
        # randoms like "test") that we should ignore.
        tags_pattern = "^\\d(\\.)",
        commit = "4.0.1",
        sha256 = "7a1c3f157aa3b9e0973e957ac0b402c09a83d405247d278c10eb4c390977f132",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
