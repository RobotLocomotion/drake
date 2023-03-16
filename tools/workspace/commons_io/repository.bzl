load(
    "@drake//tools/workspace:java.bzl",
    "drake_java_import",
)

def commons_io_repository(
        name,
        mirrors = None):
    drake_java_import(
        name = name,
        licenses = ["notice"],  # Apache-2.0
        local_os_targets = ["ubuntu"],
        local_jar = "/usr/share/java/commons-io.jar",
        maven_jar = "commons-io/commons-io/1.3.1/commons-io-1.3.1.jar",  # noqa
        maven_jar_sha256 = "3307319ddc221f1b23e8a1445aef10d2d2308e0ec46977b3f17cbb15c0ef335b",  # noqa
        mirrors = mirrors,
    )
