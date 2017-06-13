# -*- python -*-
load("@drake//tools:cmake_configure_file.bzl", "cmake_configure_file")
load(
    "@drake//tools:generate_include_header.bzl",
    "drake_generate_include_header",
)
load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)

package(default_visibility = ["//visibility:public"])

# Generates config.hh based on the version numbers in CMake code.
cmake_configure_file(
    name = "config",
    src = "cmake/config.hh.in",
    out = "include/ignition/rndf/config.hh",
    cmakelists = ["CMakeLists.txt"],
    defines = [
        # It would be nice to get this information directly from
        # CMakeLists.txt, but it ends up being too hard. We'd have to extend
        # the cmake_configure_file functionality to create variables from
        # "project" command as well as manipulate strings, and I'm not sure it
        # is worth it.  We just hard code some values that should not change.
        "PROJECT_NAME=ignition-rndf",
        "PROJECT_NAME_LOWER=ignition-rndf",
    ],
    visibility = ["//visibility:private"],
)

public_headers = glob([
    "include/**/*.hh",
])

# Generates rndf.hh, which consists of #include statements for all of the
# public headers in the library.  The first line is
# '#include <ignition/rndf/config.hh>' followed by one line like
# '#include <ignition/rndf/Checkpoint.hh>' for each non-generated header.
drake_generate_include_header(
    name = "rndfhh_genrule",
    out = "include/ignition/rndf.hh",
    hdrs = [":config"] + public_headers,
    visibility = ["//visibility:private"],
)

cc_library(
    name = "ignition_rndf",
    srcs = glob(
        ["src/*.cc"],
        exclude = ["src/*_TEST.cc"],
    ),
    hdrs = public_headers,
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ignition_math",
    ],
)

CMAKE_PACKAGE = "ignition-rndf0"

cmake_config(
    package = CMAKE_PACKAGE,
    script = "@drake//tools:ignition_rndf-create-cps.py",
    version_file = "CMakeLists.txt",
    deps = ["@ignition_math//:cps"],
)

install_cmake_config(package = CMAKE_PACKAGE)  # Creates rule :install_cmake_config.

install(
    name = "install",
    hdrs = public_headers + [
        ":config",
        ":rndfhh_genrule",
    ],
    doc_dest = "share/doc/" + CMAKE_PACKAGE,
    hdr_dest = "include",
    hdr_strip_prefix = ["include"],
    license_docs = [
        "LICENSE",
        "COPYING",
    ],
    targets = [":ignition_rndf"],
    deps = [":install_cmake_config"],
)
