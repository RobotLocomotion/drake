load("//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "b474ba15c0f0487c2944a79ba880549dba291457",
        sha256 = "d5cf0a3567392e5c3f0a30bbda852a30d945eced6858b3d6749226d02d5ef7eb",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/sre_deprecation.patch",
            ":patches/test_paths.patch",
            ":patches/include_for_fmt_join.patch",
        ],
        mirrors = mirrors,
    )
