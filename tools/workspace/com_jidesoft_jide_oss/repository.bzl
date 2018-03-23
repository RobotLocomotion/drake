# -*- mode: python -*-

load("@bazel_tools//tools/build_defs/repo:java.bzl", "java_import_external")

def com_jidesoft_jide_oss_repository(
        name,
        mirrors = None):
    java_import_external(
        name = name,
        licenses = ["restricted"],  # GPL-2.0 WITH Classpath-exception-2.0
        jar_urls = [
            x.format(fulljar = "com/jidesoft/jide-oss/2.9.7/jide-oss-2.9.7.jar")  # noqa
            for x in mirrors.get("maven")
        ],
        jar_sha256 = "a2edc2749cf482f6b2b1331f35f0383a1a11c19b1cf6d9a8cf7c69ce4cc8e04b",  # noqa
    )
