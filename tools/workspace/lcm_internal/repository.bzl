load("//tools/workspace:github.bzl", "github_archive")

def lcm_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        upgrade_advice = """
        When updating, lcm needs its own pull request separate from the rest of
        the monthly upgrades.
        """,
        # TODO(jwnimmer-tri) At the moment we have both @lcm and @lcm_internal
        # repositories, with @lcm being deprecated 2026-02-01. Be aware that we
        # have different pins for each repository.
        commit = "v1.5.2",
        sha256 = "d443261619080f1c0693237b2019436988e1b2b2ba5fc09a49bf23769e1796de",  # noqa
        patches = [
            ":patches/maven.patch",
            ":patches/vendor_namespace.patch",
        ],
        patch_cmds = [
            "echo 'exports_files([\"drake_repository_metadata.json\"])' >> BUILD.bazel",  # noqa
        ],
        mirrors = mirrors,
    )
