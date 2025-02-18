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

        To calculate the checksums, download the deb files specifed in
        install_bazelisk.sh and use:
          shasum -a 256 'xxx.deb'

        To fully test, a Linux uprovisioned job must be launched from the
        pull request.
        """,
        commit = "v1.25.0",
        sha256 = "8ff4c6b9ab6a00fbef351d52fde39afc2b9f047865f219a89ed0b23ad6f8cf06",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
