load("//tools/workspace:github.bzl", "github_archive")

def bazelisk_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazelisk",
        upgrade_advice = """
        When upgrading, most Linux uprovisioned jobs should be launched from
        the pull request. See the jenkins-jobs-experimental branch on
        RobotLocomotion/drake for job lists.
        """,  # noqa
        upgrade_type = "release",
        post_upgrade_script = "upgrade.py",
        # Our upgrade.py modifies things outside of the current directory.
        extra_upgrade_paths = [
            "setup/ubuntu",
            "third_party/com_github_bazelbuild_bazelisk",
        ],
        commit = "v1.29.0",
        sha256 = "7e4c7b8ade016052e63c1553cb4fbe0c4fe921e1e66913d49eef074ed894e933",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
