# -*- mode: python -*-

load("@bazel_tools//tools/build_defs/repo:java.bzl", "java_import_external")
load("@drake//tools/workspace:maven.bzl", "MAVEN_BASE_URLS")

def org_apache_xmlgraphics_commons_repository(name):
    java_import_external(
        name = name,
        licenses = [],
        jar_urls = [
            base + "org/apache/xmlgraphics/xmlgraphics-commons/1.3.1/xmlgraphics-commons-1.3.1.jar"  # noqa
            for base in MAVEN_BASE_URLS
        ],
        jar_sha256 = "7ce0c924c84e2710c162ae1c98f5047d64f528268792aba642d4bae5e1de7181",  # noqa
    )
