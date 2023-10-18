load("//tools/workspace:github.bzl", "github_archive")

def stduuid_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mariusbancila/stduuid",
        commit = "v1.2.3",
        sha256 = "b1176597e789531c38481acbbed2a6894ad419aab0979c10410d59eb0ebf40d3",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
