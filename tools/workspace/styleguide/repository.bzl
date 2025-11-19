load("//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "6cc89c4a3f4bf4189bbea727d04b5762517bc3f2",
        sha256 = "72618239fee8e4619c85e897ae8a3c94d93f33ae1c09ed12f79b84d0d7b7bd58",  # noqa
        # TODO(jwnimmer-tri) Simplify on 2025-06-01 during deprecation removal.
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel" if "internal" in name else ":package-deprecated.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
