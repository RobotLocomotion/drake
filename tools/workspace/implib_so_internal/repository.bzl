load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "TaylorSasser/Implib.so",
        commit = "3169e35996c349b392e5dad305913101ab3a7c21",
        sha256 = "43dfa678bfc69f7f4c632c2826bc4450dacc4de279db1a83d0004fa64f2a06d0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
