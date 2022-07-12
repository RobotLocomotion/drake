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
        commit = "115e12323529d524786c1a744f5ffce04f4783b5",  # ibex-2.8.6_4
        commit_pin = True,
        sha256 = "3cc12cfffc24d9dff8dbe8c7ef48ebbae14bc8b2548a9ff778c5582ca7adf70c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            # pkgconfig provides the clp/coin directory as an include, not clp/
            ":clp_include_path.patch",
            ":include_limits.patch",
        ],
    )
