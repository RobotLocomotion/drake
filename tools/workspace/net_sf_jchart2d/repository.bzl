# -*- mode: python -*-

load("@bazel_tools//tools/build_defs/repo:java.bzl", "java_import_external")

def net_sf_jchart2d_repository(
        name,
        mirrors = None):
    # In the unlikely event that you update the version here, verify that the
    # licenses in tools/third_party/jchart2d/LICENSE are still applicable, and
    # fix up the two jchart2d-*.cmake files in this directory.
    java_import_external(
        name = name,
        licenses = ["restricted"],  # LGPL-3.0+
        jar_urls = [
            x.format(fulljar = "net/sf/jchart2d/jchart2d/3.3.2/jchart2d-3.3.2.jar")  # noqa
            for x in mirrors.get("maven")
        ],
        jar_sha256 = "41af674b1bb00d8b89a0649ddaa569df5750911b4e33f89b211ae82e411d16cc",  # noqa
    )
