# -*- python -*-

load("@drake//tools:cmake_configure_file.bzl", "cmake_configure_file")
load("@drake//tools:install.bzl", "cmake_config", "install", "install_cmake_config")

package(default_visibility = ["//visibility:public"])

# Generates sdf_config.h based on the version numbers in CMake code.
cmake_configure_file(
    name = "config",
    src = "cmake/sdf_config.h.in",
    out = "include/sdf/sdf_config.h",
    cmakelists = ["CMakeLists.txt"],
    defines = [
        "PROJECT_NAME=SDFormat",
        "SDF_VERSION_NAME=",
        # The sdformat library currently requires that it be able to find the
        # SDF files that are installed in the cc_library :data variable, which
        # is why we set the DATAROOTDIR below.
        # TODO: change the sdformat library to not require this, or use bazel
        # variables to find the correct location to place these files.
        "CMAKE_INSTALL_FULL_DATAROOTDIR=external/sdformat/sdf/1.6",
        "SDF_PKG_VERSION=",
    ],
    visibility = ["//visibility:private"],
)

public_headers = [
    "include/sdf/Assert.hh",
    "include/sdf/Console.hh",
    "include/sdf/Element.hh",
    "include/sdf/Exception.hh",
    "include/sdf/Filesystem.hh",
    "include/sdf/Param.hh",
    "include/sdf/parser.hh",
    "include/sdf/SDFImpl.hh",
    "include/sdf/system_util.hh",
    "include/sdf/Types.hh",
]

# Generates sdf.hh, which consists of #include statements for all of the public
# headers in the library.  There is one line like '#include <sdf/Assert.hh>'
# for each non-generated header, followed at the end by a
# single '#include <sdf/sdf_config.h>'.
genrule(
    name = "sdfhh_genrule",
    srcs = public_headers,
    outs = ["include/sdf/sdf.hh"],
    # TODO: We should centralize this logic, as it is used here, in
    # ignition_math.BUILD, and in fcl.BUILD.
    cmd = "(" + (
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\<\\1\\>|g' &&" +
        "echo '#include <sdf/sdf_config.h>'"
    ) + ") > '$@'",
    visibility = ["//visibility:private"],
)

# Generates the library exported to users.  The explicitly listed srcs= matches
# upstream's explicitly listed sources plus private headers.  The explicitly
# listed hdrs= matches upstream's public headers.
cc_library(
    name = "sdformat",
    srcs = [
        "include/sdf/Converter.hh",
        "include/sdf/ExceptionPrivate.hh",
        "include/sdf/SDFExtension.hh",
        "include/sdf/SDFImplPrivate.hh",
        "include/sdf/parser_private.hh",
        "include/sdf/parser_urdf.hh",
        "include/sdf/sdf.hh",  # from genrule above
        "include/sdf/sdf_config.h",  # from cmake_configure_file above
        "src/Console.cc",
        "src/Converter.cc",
        "src/Element.cc",
        "src/Exception.cc",
        "src/Filesystem.cc",
        "src/Param.cc",
        "src/SDF.cc",
        "src/SDFExtension.cc",
        "src/Types.cc",
        "src/parser.cc",
        "src/parser_urdf.cc",
        "src/urdf/urdf_exception/exception.h",
        "src/urdf/urdf_model/color.h",
        "src/urdf/urdf_model/joint.h",
        "src/urdf/urdf_model/link.h",
        "src/urdf/urdf_model/model.h",
        "src/urdf/urdf_model/pose.h",
        "src/urdf/urdf_model/twist.h",
        "src/urdf/urdf_model/types.h",
        "src/urdf/urdf_model/utils.h",
        "src/urdf/urdf_model_state/model_state.h",
        "src/urdf/urdf_model_state/twist.h",
        "src/urdf/urdf_model_state/types.h",
        "src/urdf/urdf_parser/exportdecl.h",
        "src/urdf/urdf_parser/joint.cpp",
        "src/urdf/urdf_parser/link.cpp",
        "src/urdf/urdf_parser/model.cpp",
        "src/urdf/urdf_parser/pose.cpp",
        "src/urdf/urdf_parser/twist.cpp",
        "src/urdf/urdf_parser/urdf_model_state.cpp",
        "src/urdf/urdf_parser/urdf_parser.h",
        "src/urdf/urdf_parser/urdf_sensor.cpp",
        "src/urdf/urdf_parser/world.cpp",
        "src/urdf/urdf_sensor/sensor.h",
        "src/urdf/urdf_sensor/types.h",
        "src/urdf/urdf_world/types.h",
        "src/urdf/urdf_world/world.h",
    ],
    hdrs = public_headers,
    # TODO: We are currently using the vendored version of urdfdom embedded
    # in sdformat, so we need this include path to find it.  We can get
    # rid of this by either building the vendored version as a separate
    # cc_library rule, or by using a true external version of URDF.
    copts = ["-I external/sdformat/src/urdf"],
    data = glob(["sdf/1.6/*.sdf"]),
    includes = [
        "include",
    ],
    linkopts = [
        "-lboost_system",
        "-ltinyxml",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@ignition_math",
    ],
)

cmake_config(
    package = "SDFormat",
    script = "@drake//tools:sdformat-create-cps.py",
    version_file = "CMakeLists.txt",
    deps = ["@ignition_math//:cps"],
)

install_cmake_config(package = "SDFormat")  # Creates rule :install_cmake_config.

install(
    name = "install",
    hdrs = public_headers + [
        ":sdfhh_genrule",
        ":config",
    ],
    doc_dest = "share/doc/sdformat",
    hdr_dest = "include",
    hdr_strip_prefix = ["include"],
    license_docs = ["LICENSE", "COPYING"],
    targets = [":sdformat"],
    deps = [":install_cmake_config"],
)
