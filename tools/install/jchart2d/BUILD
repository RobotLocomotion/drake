# -*- python -*-

load("//tools:install.bzl", "cmake_config", "install", "install_cmake_config")

package(default_visibility = ["//visibility:public"])

CMAKE_PACKAGE = "jchart2d"

cmake_config(
    cps_file_name = CMAKE_PACKAGE + ".cps",
    package = CMAKE_PACKAGE,
)

install_cmake_config(package = CMAKE_PACKAGE)

JCHART_LICENSE_DOCS = ["//tools:third_party/jchart2d/LICENSE"]

JCHART_TARGETS = [
    "@com_jidesoft_jide_oss//jar",
    "@commons_io_commons_io//jar",
    "@net_sf_jchart2d_jchart2d//jar",
    "@org_apache_xmlgraphics_xmlgraphics_commons//jar",
]

install(
    name = "install",
    allowed_externals = JCHART_LICENSE_DOCS + JCHART_TARGETS,
    doc_dest = "share/doc/" + CMAKE_PACKAGE,
    doc_strip_prefix = ["**/"],
    license_docs = JCHART_LICENSE_DOCS,
    java_strip_prefix = ["**/"],
    targets = JCHART_TARGETS,
    deps = [":install_cmake_config"],
    visibility = ["//:__pkg__"],
)
