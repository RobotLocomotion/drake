# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def ignition_rndf_repository(name):
    bitbucket_archive(
        name = name,
        repository = "ignitionrobotics/ign-rndf",
        commit = "ignition-rndf_0.1.5",
        sha256 = "fa1033be146ff51f3b2c679ff160838c1e3ca736c565b19510a5c9b6d352fbaf",  # noqa
        strip_prefix = "ignitionrobotics-ign-rndf-214a333fbdcb",
        build_file = "@drake//tools/workspace/ignition_rndf:package.BUILD.bazel",  # noqa
    )
