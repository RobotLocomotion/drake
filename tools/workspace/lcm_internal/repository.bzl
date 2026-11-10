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
        upgrade_type = "release",
        commit = "v1.5.3",
        sha256 = "f7e693d50e229e37de00387ac8ab9228e16559dd57bddd155c4b5ca1c233834e",  # noqa
        patches = [
            ":patches/maven.patch",
            ":patches/vendor_namespace.patch",
        ],
        patch_cmds = [
            "echo 'exports_files([\"drake_repository_metadata.json\"])' >> BUILD.bazel",  # noqa
        ],
        mirrors = mirrors,
    )
