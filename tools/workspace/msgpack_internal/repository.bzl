load("//tools/workspace:github.bzl", "github_archive")

def msgpack_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "msgpack/msgpack-c",
        upgrade_type = "release",
        exclude_tags_pattern = "c-[0-9.]+",
        commit = "cpp-9.0.0",
        sha256 = "a72ff7471eb8adf8470a21aec33d70f184a20e215f167e2d4c38d56d9a377205",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
