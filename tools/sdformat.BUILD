# -*- python -*-

load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")

# Generates sdf_config.h based on the version numbers in CMake code.
cmake_configure_file(
    name = "config",
    src = "cmake/sdf_config.h.in",
    out = "include/sdf/sdf_config.h",
    cmakelists = ["CMakeLists.txt"],
    defines = [
        "PROJECT_NAME=SDFormat",
        "SDF_VERSION_NAME=",
        "CMAKE_INSTALL_FULL_DATAROOTDIR=external/sdformat/sdf/1.6",
        "SDF_PKG_VERSION=",
    ],
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
)

# Generates the library exported to users.  The explicitly listed srcs= matches
# upstream's explicitly listed sources plus private headers.  The explicitly
# listed hdrs= matches upstream's public headers.
cc_library(
    name = "sdformat",
    srcs = [
        "include/sdf/Converter.hh",
        "include/sdf/ExceptionPrivate.hh",
        "include/sdf/parser_private.hh",
        "include/sdf/parser_urdf.hh",
        "include/sdf/SDFExtension.hh",
        "include/sdf/sdf_config.h", # from cmake_configure_file above
        "include/sdf/sdf.hh",       # from genrule above
        "include/sdf/SDFImplPrivate.hh",
        "src/Console.cc",
        "src/Converter.cc",
        "src/Element.cc",
        "src/Exception.cc",
        "src/Filesystem.cc",
        "src/parser.cc",
        "src/parser_urdf.cc",
        "src/Param.cc",
        "src/SDF.cc",
        "src/SDFExtension.cc",
        "src/Types.cc",
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
        "src/urdf/urdf_parser/urdf_sensor.cpp",
        "src/urdf/urdf_parser/world.cpp",
        "src/urdf/urdf_parser/urdf_parser.h",
        "src/urdf/urdf_sensor/sensor.h",
        "src/urdf/urdf_sensor/types.h",
        "src/urdf/urdf_world/types.h",
        "src/urdf/urdf_world/world.h",
    ],
    hdrs = public_headers,
    includes = [
        "include",
    ],
    visibility = ["//visibility:public"],
    linkopts = [
        "-lboost_system",
        "-ltinyxml"
    ],
    copts = ["-I external/sdformat/src/urdf"],
    deps = [
        "@ignition_math",
    ],
    data = glob(['sdf/1.6/*.sdf']),
)
