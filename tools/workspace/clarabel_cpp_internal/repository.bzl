load("@drake//tools/workspace:github.bzl", "github_archive")

def clarabel_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oxfordcontrol/Clarabel.cpp",
        upgrade_advice = """
        When updating, any crate_universe changes should be made in the same
        commit. See tools/workspace/README.md.
        """,
        commit = "v0.6.0",
        sha256 = "281b1cbbe7e15520ad17a74d91f0ef9d83161ee79c0ff1954187f78c4516c8ec",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/extern_c.patch",
            ":patches/git_submodule.patch",
            ":patches/sdp.patch",
            ":patches/unicode.patch",
        ],
        mirrors = mirrors,
    )
