load("//tools/workspace:github.bzl", "github_archive")

def coal_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coal-library/coal",
        # Coal does not cut releases on any regular cadence, and fixes we care
        # about have landed on `devel` with no release cut for them, so we
        # track the mainline branch rather than the newest tag.
        upgrade_type = "commit",
        commit = "f0a55dfe42966369096a03b029471ec1ae5b917b",
        sha256 = "924b29ae04ff238de6473b61f903c5fdf85e3c25789c1039ca8d83a89bd30a6b",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/no_boost.patch",
            ":patches/upstream/virtual_dtor.patch",
            ":patches/eigen_no_io.patch",
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
