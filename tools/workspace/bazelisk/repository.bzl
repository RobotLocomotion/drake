load("//tools/workspace:github.bzl", "github_archive")

def bazelisk_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazelisk",
        upgrade_advice = """
        When updating, the following additional steps (run in the Drake source
        tree) must also be performed:

        $ bazel build @bazelisk//:*
        $ cp -t third_party/com_github_bazelbuild_bazelisk/ \\
            bazel-drake/external/bazelisk/LICENSE \\
            bazel-drake/external/bazelisk/bazelisk.py

        Additionally, you must manually update the version number in
          setup/ubuntu/source_distribution/install_bazelisk.sh
        and adjust the expected checksums accordingly.
        """,
        commit = "v1.22.0",
        sha256 = "4e136f6f1212f28d5c6fdd4cfa3f016d7443831fc98ce8b7ee3caee81ef956fa",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/pull494.patch",
        ],
        mirrors = mirrors,
    )
