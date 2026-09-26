load("//tools/workspace:github.bzl", "github_release_attachments")

def bazelisk_debs_internal_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "bazelbuild/bazelisk",
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        commit = "v1.29.0",
        attachments = {
            "bazelisk-amd64.deb": "186d78a20e1a64f59ba08987791a989892d142c9d3a9f9cc0c5c35e201f53924",  # noqa
            "bazelisk-arm64.deb": "db8ada89c841afd2cb33db7d13aa98ea4fb14612579a8bae84722250caa84272",  # noqa
        },
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
