load(
    "//tools/workspace/doxygen_internal:repository_impl.bzl",
    "repository_impl",
)

def doxygen_internal_repository(
        name,
        mirrors = None):
    repository_impl(
        name = name,
        repository = "doxygen/doxygen",
        commit = "Release_1_14_0",
        platform = "noble",
        sha256 = "db31e8e25e0c6eae5c90a7a291bf6b65667335c60a7ce69f7d63d1459b2ef8b7",  # noqa
        mirrors = mirrors,
    )
