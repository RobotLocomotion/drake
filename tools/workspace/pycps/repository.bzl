# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mwoehlke/pycps",
        # PR DRAFT: DO NOT MERGE
        commit = "ec0c7adc9ef660bfcc2101a79bc12a2b577bb1a9",
        sha256 = "1ee737974af204075ffb7ca5de6276972603d23ba126438b3d8acdeccc9d9096",  # noqa
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
        mirrors = mirrors,
    )
