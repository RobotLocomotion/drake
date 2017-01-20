# -*- python -*-

load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")

# Lets other packages inspect the CMake code, e.g., for the version number.
filegroup(
    name = "cmakelists_with_version",
    srcs = ["CMakeLists.txt"],
    visibility = ["//visibility:public"],
)

# Generates sdf.hh based on the version numbers in CMake code.
cmake_configure_file(
    name = "config",
    src = "cmake/sdf_config.h.in",
    out = "include/sdf/sdf_config.h",
    cmakelists = [
        ":cmakelists_with_version",
    ],
    defines = [
        "PROJECT_NAME=SDFormat",
        "SDF_VERSION_NAME=",
        "CMAKE_INSTALL_FULL_DATAROOTDIR=/usr/share",
        "SDF_PKG_VERSION=",
        "URDF_GE_0P3=1",
    ],
    visibility = [],
)

# Generates sdf.hh, which consists of #include statements for *all* of the other
# headers in the library (!!!).  There is one line like
# '#include <sdf/Assert.hh>' for each non-generated header, followed at the end
# by a single '#include <sdf/sdf_config.hh>'.
genrule(
    name = "sdfhh_genrule",
    srcs = [
        "include/sdf/Assert.hh",
        "include/sdf/Console.hh",
        "include/sdf/Element.hh",
        "include/sdf/Exception.hh",
        "include/sdf/Param.hh",
        "include/sdf/parser.hh",
        "include/sdf/SDFImpl.hh",
        "include/sdf/Types.hh",
        "include/sdf/system_util.hh",
    ],
    outs = ["include/sdf/sdf.hh"],
    cmd = "(" + (
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\<\\1\\>|g' &&" +
        "echo '#include <sdf/sdf_config.h>'"
    ) + ") > '$@'",
    visibility = [],
)

# Generates the library exported to users.  The explicitly listed srcs= matches
# upstream's explicitly listed sources.  The globbed hdrs= matches upstream's
# explicitly globbed headers.
cc_library(
    name = "lib",
    srcs = [
        "src/Assert.cc",
        "src/Console.cc",
        "src/Converter.cc",
        "src/Element.cc",
        "src/Exception.cc",
        "src/parser.cc",
        "src/parser_urdf.cc",
        "src/Param.cc",
        "src/SDF.cc",
        "src/SDFExtension.cc",
        "src/Types.cc",
        "src/urdf/urdf_parser/model.cpp",
        "src/urdf/urdf_parser/link.cpp",
        "src/urdf/urdf_parser/joint.cpp",
        "src/urdf/urdf_parser/pose.cpp",
        "src/urdf/urdf_parser/twist.cpp",
        "src/urdf/urdf_parser/urdf_model_state.cpp",
        "src/urdf/urdf_parser/urdf_sensor.cpp",
        "src/urdf/urdf_parser/world.cpp",
    ],
    hdrs = glob([
        "include/sdf/*.hh",
    ]) + [
        "include/sdf/sdf_config.h",
        "include/sdf/sdf.hh",
    ],
    includes = ["include"],
    visibility = ["//visibility:public"],
    linkopts = ["-lboost_system", "-lboost_filesystem", "-lboost_program_options", "-lboost_regex", "-lboost_iostreams", "-ltinyxml"],
    deps = [
        "@ignmath//:lib",
    ],
)
