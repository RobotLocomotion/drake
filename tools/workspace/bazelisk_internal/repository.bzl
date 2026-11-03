load("//tools/workspace:github.bzl", "github_archive")

def bazelisk_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazelisk",
        upgrade_advice = """
        When upgrading, one Linux uprovisioned job from each of our supported
        architectures (x86_64 and arm64) should be launched from the pull
        request. See the jenkins-jobs-experimental branch on
        RobotLocomotion/drake for job lists.
        """,  # noqa
        upgrade_type = "release",
        post_upgrade_script = "//tools/workspace/bazelisk_internal:upgrade",
        commit = "v1.30.0",
        sha256 = "938c875176d6895d1f9cce21d71a76f56f82e88d819f22ed587f124f795aad33",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
