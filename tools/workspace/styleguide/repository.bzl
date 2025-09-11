load("//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "63d2e8ef2a1931f014f35d1939e4e902dc57ef13",
        sha256 = "23e25b88ef9ccb916b7f5097300956147f8cb51157fbc1967f24ebe2b445c82e",  # noqa
        # TODO(jwnimmer-tri) Simplify on 2025-06-01 during deprecation removal.
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel" if "internal" in name else ":package-deprecated.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
