load("//tools/workspace:java.bzl", "drake_java_import")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

def net_sf_jchart2d_repository(
        name,
        mirrors = None,
        _is_drake_self_call = False):
    if not _is_drake_self_call:
        print_warning("net_sf_jchart2d_repository")
    drake_java_import(
        name = name,
        licenses = ["restricted"],  # LGPL-3.0+
        local_os_targets = ["linux"],
        local_jar = "/usr/share/java/jchart2d.jar",
        maven_jar = "net/sf/jchart2d/jchart2d/3.3.2/jchart2d-3.3.2.jar",  # noqa
        maven_jar_sha256 = "41af674b1bb00d8b89a0649ddaa569df5750911b4e33f89b211ae82e411d16cc",  # noqa
        mirrors = mirrors,
    )
