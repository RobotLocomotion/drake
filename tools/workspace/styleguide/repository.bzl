load("//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "81521cacb27b933acea048f37ffccd8b68769cc2",
        sha256 = "c4519d54cc30372cec99188578e419bf82e871a0e1c63fed9af376a4705b7f28",  # noqa
        # TODO(jwnimmer-tri) Simplify on 2025-06-01 during deprecation removal.
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel" if "internal" in name else ":package-deprecated.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
