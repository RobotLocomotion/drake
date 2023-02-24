# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RussTedrake/models",
        commit = "d37a245b32f32127aa24442eb72197911bd346cb",
        sha256 = "02882d038d33c96bdfa056b161884fd36d9b61d4c163828519f7fe82ebded84e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
