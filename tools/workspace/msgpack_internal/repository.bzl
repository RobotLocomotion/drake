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
        commit = "cpp-6.0.0",
        sha256 = "d02f7ffd28b1d38ab9f5f758c4744fadfae92150461fb8154c98ac49226cff90",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
