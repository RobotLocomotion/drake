load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "TaylorSasser/Implib.so",
        upgrade_type = "commit",
        commit = "ead05abb7d1e70d2a412a463208ddbe881037e37",
        sha256 = "bef5299700faab5454dc72dd1f71f779330c8bca149a3e1fa7c40d4c159bca9b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
