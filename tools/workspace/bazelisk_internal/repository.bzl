load("//tools/workspace:github.bzl", "github_archive")

def bazelisk_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazelisk",
        upgrade_advice = """
        When updating, the following additional steps (run in the Drake source
        tree) must also be performed:

        $ bazel build @bazelisk_internal//:*
        $ cp -t third_party/com_github_bazelbuild_bazelisk/ \\
            bazel-drake/external/+internal_repositories+bazelisk_internal/LICENSE \\
            bazel-drake/external/+internal_repositories+bazelisk_internal/bazelisk.py

        Additionally, you must manually update the version number in
          setup/ubuntu/source_distribution/install_bazelisk.sh
        and adjust the expected checksums accordingly.

        To calculate the checksums, download the deb files specifed in
        install_bazelisk.sh and use:
          shasum -a 256 'xxx.deb'

        To fully test, a Linux uprovisioned job must be launched from the
        pull request.
        """,  # noqa
        commit = "v1.28.1",
        sha256 = "e80f76b9d86f529e9d267ce0d333365ea14ec92b3269f81ab85cbd69edab2793",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
