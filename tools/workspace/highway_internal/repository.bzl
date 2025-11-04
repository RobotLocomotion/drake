load("//tools/workspace:github.bzl", "github_archive")

def highway_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/highway",
        commit = "1.3.0",
        sha256 = "07b3c1ba2c1096878a85a31a5b9b3757427af963b1141ca904db2f9f4afe0bc2",  # noqa
        patches = [
            ":patches/upstream/build.patch",
            ":patches/disabled_targets.patch",
            ":patches/linkstatic.patch",
            ":patches/target_get_index_inline_always.patch",
            ":patches/target_update_noinline.patch",
        ],
        patch_cmds = [
            "echo 'exports_files([\"drake_repository_metadata.json\"])' >> BUILD",  # noqa
        ],
        mirrors = mirrors,
    )
