load("//tools/workspace:github.bzl", "github_archive")

def msgpack_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "msgpack/msgpack-c",
        upgrade_type = "release",
        exclude_tags_pattern = "c-[0-9.]+",
        commit = "cpp-8.0.0",
        sha256 = "f634fb7052da4478096f2a02dfb6d91174e5836b317afb006375249ccb086aa8",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
