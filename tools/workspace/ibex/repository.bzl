# -*- mode: python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ibex_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # TODO(jwnimmer-tri) Switch to upstream ibex-lib (using local patch
        # files if necessary).
        repository = "dreal-deps/ibex-lib",
        # As discussed in #15872, we need ibex < 2.8.7 for CLP support.
        commit = "ibex-2.8.6_4",
        commit_pin = True,
        sha256 = "172f2cf8ced69bd2e30be448170655878735af7d0bf6d2fef44b14215c8b1a49",  # noqa
        build_file = "@drake//tools/workspace/ibex:package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            # pkgconfig provides the clp/coin directory as an include, not clp/
            "@drake//tools/workspace/ibex:clp_include_path.patch",
        ],
    )
