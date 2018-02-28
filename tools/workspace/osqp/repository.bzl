# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def osqp_repository(name):
    github_archive(
        name = name,
        repository = "oxfordcontrol/osqp",
        commit = "c1d13d4b499243ea4afa0a321aa2226c6c1cd29a",
        sha256 = "8886fbaa794effcc39bb11c2cb089b247de552e95b9ccdcda7baa9d973507ba4",  # noqa
        build_file = "@drake//tools/workspace/osqp:package.BUILD.bazel",
    )
