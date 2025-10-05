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
        # repositories. We are working to deprecate @lcm. In the meantime, be
        # aware that we have a different pins for each repository.
        # TODO(jwnimmer-tri) Once LCM has its next tagged release >v1.5.1, we
        # should switch this back to a release tag instead of this hash.
        commit = "e4bed2c86fbd6dd2280326801acf71cbd05074be",
        sha256 = "b2bf5bf7fed61805d72855c8ea9d247de95e5ca885ee6c5c8c9185aa87dda74c",  # noqa
        patches = [
            ":patches/copts.patch",
            ":patches/vendor_namespace.patch",
        ],
        patch_cmds = [
            "echo 'exports_files([\"drake_repository_metadata.json\"])' >> BUILD.bazel",  # noqa
        ],
        mirrors = mirrors,
    )
