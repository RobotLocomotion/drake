# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

# TODO(jwnimmer-tri) For easier ugprades, stop using the dreal-deps mirror and
# switch to using the upstream archive at http://fmv.jku.at/picosat/.

def picosat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "dreal-deps/picosat",  # License: MIT
        commit = "4ee7aa1d1c645df8fa9daa07f2be17c6d03b35fc",  # v965
        sha256 = "1be461d3659d4e3dc957a718ed295941c38dc822fd22a67f4cb5d180f0b6a7a3",  # noqa
        build_file = "@dreal//tools:picosat.BUILD.bazel",
        mirrors = mirrors,
    )
