# -*- mode: python -*-

load("@bazel_tools//tools/build_defs/repo:java.bzl", "java_import_external")

def org_apache_xmlgraphics_commons_repository(
        name,
        mirrors = None):
    java_import_external(
        name = name,
        licenses = ["notice"],  # Apache-2.0
        jar_urls = [
            x.format(fulljar = "org/apache/xmlgraphics/xmlgraphics-commons/1.3.1/xmlgraphics-commons-1.3.1.jar")  # noqa
            for x in mirrors.get("maven")
        ],
        jar_sha256 = "7ce0c924c84e2710c162ae1c98f5047d64f528268792aba642d4bae5e1de7181",  # noqa
    )
