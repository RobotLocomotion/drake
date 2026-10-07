load("//tools/workspace:github.bzl", "github_archive")

def fcl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        upgrade_type = "commit",
        commit = "708d2e102c12968557b70bf6785d473456310de5",
        sha256 = "86cbeea09fa06159cc2518f9433ed97eeebe87e65c6a2bc78911a656744561bb",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/thread_safe_collision_object_copy_constructor.patch",
        ],
        mirrors = mirrors,
    )
