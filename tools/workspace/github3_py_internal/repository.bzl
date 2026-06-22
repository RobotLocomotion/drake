load("//tools/workspace:github.bzl", "github_archive")

def github3_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sigmavirus24/github3.py",
        upgrade_type = "tag",
        upgrade_advice = """
        This upgrade requires manual testing. Check the following for reliance
        on any changed APIs:
          * //tools/release_engineering:mirror_to_s3
          * //tools/release_engineering:relnotes
          * //tools/workspace:new_release
        If unsure on how to test these changes locally or in CI, contact
        @BetsyMcPhail.
        """,
        exclude_tags_pattern = "^[^\\d].*",
        commit = "4.0.1",
        sha256 = "7a1c3f157aa3b9e0973e957ac0b402c09a83d405247d278c10eb4c390977f132",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
