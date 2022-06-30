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
        commit = "a37ff03c5f78678f06801f501b54b1b04f662374",  # v965
        sha256 = "e1d417c22c61b9d8344d4d5ad4132cf7bc25bb45e6434b97d6d134d1cb80fbb1",  # noqa
        build_file = "@dreal//tools:picosat.BUILD.bazel",
        patches = [
            "@drake//tools/workspace/picosat:patches/ubsan.patch",
        ],
        mirrors = mirrors,
    )
