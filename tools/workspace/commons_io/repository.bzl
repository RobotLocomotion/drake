# -*- mode: python -*-

load("@bazel_tools//tools/build_defs/repo:java.bzl", "java_import_external")

def commons_io_repository(
        name,
        mirrors = None):
    java_import_external(
        name = name,
        licenses = ["notice"],  # Apache-2.0
        jar_urls = [
            x.format(fulljar = "commons-io/commons-io/1.3.1/commons-io-1.3.1.jar")  # noqa
            for x in mirrors.get("maven")
        ],
        jar_sha256 = "3307319ddc221f1b23e8a1445aef10d2d2308e0ec46977b3f17cbb15c0ef335b",  # noqa
    )
