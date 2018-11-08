# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def qdldl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oxfordcontrol/qdldl",
        # When changing the commit of QDLDL used by Drake, ideally try to keep
        # it aligned with what Drake's commit of OSQP desires, e.g.,
        # https://github.com/oxfordcontrol/osqp/tree/v0.4.1/lin_sys/direct/qdldl
        # shows that v0.4.1 of OSQP prefers v0.1.3 of QDLDL.
        #
        # Note that the "commit" listed below is akin to v0.1.3, except that
        # upstream created a branch instead of a tag for this release.
        commit = "b145d782e88cc2308c54bf0c9f221d9f6597082c",
        sha256 = "af60253edab75af049cbb8c66a2bf1a36f2362125429fcfe2adf860dc20ef201",  # noqa
        build_file = "@drake//tools/workspace/qdldl:package.BUILD.bazel",
        mirrors = mirrors,
    )
