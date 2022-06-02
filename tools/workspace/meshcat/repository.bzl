# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "joemasterjohn/meshcat",
        # Updating this commit requires local testing; see
        # drake/tools/workspace/meshcat/README.md for details.
	commit = "359cf3af8ed41695c98ffcb47a58a87801210aa1",
	sha256 = "27e47738810b40d234d01cd44edd52acf47efd65660e5de0e98fa7964aef2ed8",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
