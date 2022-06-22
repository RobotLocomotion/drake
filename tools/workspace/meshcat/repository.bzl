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
	commit = "5c1398ac9c6453ddd33204589f2162c39d6d2c5f",
	sha256 = "27adc6dcb9e8aaee00b577a2d46541a99e177178576df6bad5a06d6677ec49c2",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
