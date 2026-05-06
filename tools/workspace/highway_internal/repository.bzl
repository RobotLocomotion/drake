load("//tools/workspace:github.bzl", "github_archive")

def highway_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/highway",
        commit = "1.4.0",
        sha256 = "e72241ac9524bb653ae52ced768b508045d4438726a303f10181a38f764a453c",  # noqa
        patches = [
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
