# -*- mode: python -*-

load("@bazel_tools//tools/build_defs/repo:java.bzl", "java_import_external")
load("@drake//tools/workspace:maven.bzl", "MAVEN_BASE_URLS")

def commons_io_repository(name):
    java_import_external(
        name = name,
        licenses = [],
        jar_urls = [
            base + "commons-io/commons-io/1.3.1/commons-io-1.3.1.jar"
            for base in MAVEN_BASE_URLS
        ],
        jar_sha256 = "3307319ddc221f1b23e8a1445aef10d2d2308e0ec46977b3f17cbb15c0ef335b",  # noqa
    )
