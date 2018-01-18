# -*- mode: python -*-

load("@bazel_tools//tools/build_defs/repo:java.bzl", "java_import_external")
load("@drake//tools/workspace:maven.bzl", "MAVEN_BASE_URLS")

def net_sf_jchart2d_repository(name):
    # In the unlikely event that you update the version here, verify that the
    # licenses in tools/third_party/jchart2d/LICENSE are still applicable.
    java_import_external(
        name = name,
        licenses = [],
        jar_urls = [
            base + "net/sf/jchart2d/jchart2d/3.3.2/jchart2d-3.3.2.jar"
            for base in MAVEN_BASE_URLS
        ],
        jar_sha256 = "41af674b1bb00d8b89a0649ddaa569df5750911b4e33f89b211ae82e411d16cc",  # noqa
    )
