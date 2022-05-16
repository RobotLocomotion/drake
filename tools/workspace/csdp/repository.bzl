# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def csdp_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Csdp",
        commit = "releases/6.2.0",
        sha256 = "3d341974af1f8ed70e1a37cc896e7ae4a513375875e5b46db8e8f38b7680b32f",  # noqa
        build_file = "@drake//tools/workspace/csdp:package.BUILD.bazel",
        patches = [
            "@drake//tools/workspace/csdp:patches/params_pathname.patch",
            "@drake//tools/workspace/csdp:patches/printlevel.patch",
        ],
        patch_cmds = [
            # Move the headers into a subdirectory, so they don't pollute the
            # top-level include path.
            "mkdir includes",
            "mv include includes/csdp",
            "sed -i -e 's|^#include \"|#include \"csdp/|g;' lib/*.c",
            # Add include guards.
            "sed -i -e $'1s/^/#pragma once\\\n/' includes/csdp/*.h",
        ],
        mirrors = mirrors,
    )
