load("//tools/workspace:java.bzl", "drake_java_import")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

def org_apache_xmlgraphics_commons_repository(
        name,
        mirrors = None,
        _is_drake_self_call = False):
    if not _is_drake_self_call:
        print_warning("org_apache_xmlgraphics_commons_repository")
    drake_java_import(
        name = name,
        licenses = ["notice"],  # Apache-2.0
        local_os_targets = ["linux"],
        local_jar = "/usr/share/java/xmlgraphics-commons.jar",
        maven_jar = "org/apache/xmlgraphics/xmlgraphics-commons/1.3.1/xmlgraphics-commons-1.3.1.jar",  # noqa
        maven_jar_sha256 = "7ce0c924c84e2710c162ae1c98f5047d64f528268792aba642d4bae5e1de7181",  # noqa
        mirrors = mirrors,
    )
