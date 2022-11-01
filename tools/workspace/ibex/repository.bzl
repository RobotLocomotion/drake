# -*- mode: python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

# N.B. This repository is deprecated for removal on 2023-02-01.
# For details see https://github.com/RobotLocomotion/drake/pull/18156.

def ibex_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
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
