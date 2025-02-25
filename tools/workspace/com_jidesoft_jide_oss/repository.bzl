load("//tools/workspace:java.bzl", "drake_java_import")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

def com_jidesoft_jide_oss_repository(
        name,
        mirrors = None,
        _is_drake_self_call = False):
    if not _is_drake_self_call:
        print_warning("com_jidesoft_jide_oss_repository")
    drake_java_import(
        name = name,
        licenses = ["restricted"],  # GPL-2.0 WITH Classpath-exception-2.0
        local_os_targets = ["linux"],
        local_jar = "/usr/share/java/jide-oss.jar",
        maven_jar = "com/jidesoft/jide-oss/2.9.7/jide-oss-2.9.7.jar",  # noqa
        maven_jar_sha256 = "a2edc2749cf482f6b2b1331f35f0383a1a11c19b1cf6d9a8cf7c69ce4cc8e04b",  # noqa
        mirrors = mirrors,
    )
