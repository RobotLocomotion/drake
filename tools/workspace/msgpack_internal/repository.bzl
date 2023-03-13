load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def msgpack_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "msgpack/msgpack-c",
        commit = "cpp-5.0.0",
        sha256 = "bd6b8e255f0a62cf8f50f1d292f979ac8ea9a4aa121938679d6f419d6df70ea3",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
