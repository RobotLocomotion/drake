load("//tools/workspace:github.bzl", "github_archive")

def fcl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        upgrade_type = "commit",
        commit = "ae86ecde788086110f8a4cea54ae1949df8a21d3",
        sha256 = "4fa2e0862f5b378379cb17725666de02af70d72ad06d96db0b6df8fb596b5c5a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/thread_safe_collision_object_copy_constructor.patch",
        ],
        mirrors = mirrors,
    )
