load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.3.1",
        sha256 = "b512484396a80750acf3082adc1807ba0aabb103c2e09be5691f46f14d0a9718",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
