# -*- mode: python -*-

load("@bazel_tools//tools/build_defs/repo:java.bzl", "java_import_external")
load("@drake//tools/workspace:maven.bzl", "MAVEN_BASE_URLS")

def com_jidesoft_jide_oss_repository(name):
    java_import_external(
        name = name,
        licenses = [],
        jar_urls = [
            base + "com/jidesoft/jide-oss/2.9.7/jide-oss-2.9.7.jar"
            for base in MAVEN_BASE_URLS
        ],
        jar_sha256 = "a2edc2749cf482f6b2b1331f35f0383a1a11c19b1cf6d9a8cf7c69ce4cc8e04b",  # noqa
    )
