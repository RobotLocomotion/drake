# -*- python -*-

load("@drake//tools:cmake_configure_file.bzl", "cmake_configure_file")
load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)
load("@drake//tools:python_lint.bzl", "python_lint")

package(default_visibility = ["//visibility:public"])

# Chooses the nlopt preprocessor substitutions that we want to use from Bazel.
cmake_configure_file(
    name = "config",
    src = "nlopt_config.h.in",
    out = "nlopt_config.h",
    cmakelists = ["CMakeLists.txt"],
    defines = [
        # These end up being unused; empty-string is a fail-fast value.
        "SIZEOF_UNSIGNED_INT=",
        "SIZEOF_UNSIGNED_LONG=",
        # C11 standard spelling.
        "THREADLOCAL=_Thread_local",
        # Say "yes" to everything; we only use modern platforms from Bazel.
        "HAVE_COPYSIGN=1",
        "HAVE_DLFCN_H=1",
        "HAVE_FPCLASSIFY=1",
        "HAVE_GETOPT_H=1",
        "HAVE_GETPID=1",
        "HAVE_GETTIMEOFDAY=1",
        "HAVE_INTTYPES_H=1",
        "HAVE_ISINF=1",
        "HAVE_ISNAN=1",
        "HAVE_MEMORY_H=1",
        "HAVE_QSORT_R=1",
        "HAVE_STDINT_H=1",
        "HAVE_STDLIB_H=1",
        "HAVE_STRINGS_H=1",
        "HAVE_STRING_H=1",
        "HAVE_SYS_STAT_H=1",
        "HAVE_SYS_TYPES_H=1",
        "HAVE_SYS_TIME_H=1",
        "HAVE_TIME=1",
        "HAVE_UINT32_T=1",
        "HAVE_UNISTD_H=1",
        "TIME_WITH_SYS_TIME=1",
        # Yes, we are going to build the C++ bindings.
        "WITH_CXX=1",
    ],
    visibility = ["//visibility:private"],
)

# Creates api/nlopt.hpp based on api/nlopt.h.
genrule(
    name = "nlopt_hpp_genrule",
    srcs = [
        "api/nlopt-in.hpp",
        "api/nlopt.h",
    ],
    outs = ["api/nlopt.hpp"],
    cmd = "$(location @drake//tools:nlopt-gen-hpp.sh) $(SRCS) $(OUTS)" +
          " 2>&1 1>log || (cat log && false)",
    tools = ["@drake//tools:nlopt-gen-hpp.sh"],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "nlopt",
    srcs = [
        # This comes from a genrule above.
        "nlopt_config.h",
    ] + [
        # This list exactly matches CMakeLists.txt NLOPT_SOURCES.
        "direct/DIRect.c",
        "direct/direct_wrap.c",
        "direct/DIRserial.c",
        "direct/DIRsubrout.c",
        "direct/direct-internal.h",
        "direct/direct.h",
        "cdirect/cdirect.c",
        "cdirect/hybrid.c",
        "cdirect/cdirect.h",
        "praxis/praxis.c",
        "praxis/praxis.h",
        "luksan/plis.c",
        "luksan/plip.c",
        "luksan/pnet.c",
        "luksan/mssubs.c",
        "luksan/pssubs.c",
        "luksan/luksan.h",
        "crs/crs.c",
        "crs/crs.h",
        "mlsl/mlsl.c",
        "mlsl/mlsl.h",
        "mma/mma.c",
        "mma/mma.h",
        "mma/ccsa_quadratic.c",
        "cobyla/cobyla.c",
        "cobyla/cobyla.h",
        "newuoa/newuoa.c",
        "newuoa/newuoa.h",
        "neldermead/nldrmd.c",
        "neldermead/neldermead.h",
        "neldermead/sbplx.c",
        "auglag/auglag.c",
        "auglag/auglag.h",
        "bobyqa/bobyqa.c",
        "bobyqa/bobyqa.h",
        "isres/isres.c",
        "isres/isres.h",
        "slsqp/slsqp.c",
        "slsqp/slsqp.h",
        "esch/esch.c",
        "esch/esch.h",
        "api/general.c",
        "api/options.c",
        "api/optimize.c",
        "api/deprecated.c",
        "api/nlopt-internal.h",
        "api/nlopt.h",
        "api/f77api.c",
        "api/f77funcs.h",
        "api/f77funcs_.h",
        "util/mt19937ar.c",
        "util/sobolseq.c",
        "util/soboldata.h",
        "util/timer.c",
        "util/stop.c",
        "util/nlopt-util.h",
        "util/redblack.c",
        "util/redblack.h",
        "util/qsort_r.c",
        "util/rescale.c",
        # This list exactly matches CMakeLists.txt additions for WITH_CXX.
        "stogo/global.cc",
        "stogo/linalg.cc",
        "stogo/local.cc",
        "stogo/stogo.cc",
        "stogo/tools.cc",
        "stogo/global.h",
        "stogo/linalg.h",
        "stogo/local.h",
        "stogo/stogo_config.h",
        "stogo/stogo.h",
        "stogo/tools.h",
    ],
    hdrs = [
        "api/nlopt.h",
        ":nlopt_hpp_genrule",
    ],
    copts = [
        "-Wno-all",
        "-Wno-deprecated-declarations",
    ],
    includes = [
        ".",
        # This list exactly matches CMakeLists.txt include_directories.
        "stogo",
        "util",
        "direct",
        "cdirect",
        "praxis",
        "luksan",
        "crs",
        "mlsl",
        "mma",
        "cobyla",
        "newuoa",
        "neldermead",
        "auglag",
        "bobyqa",
        "isres",
        "slsqp",
        "esch",
        "api",
    ],
)

cmake_config(
    package = "NLopt",
    script = "@drake//tools:nlopt-create-cps.py",
    version_file = "nlopt_config.h",
)

install_cmake_config(package = "NLopt")  # Creates rule :install_cmake_config.

install(
    name = "install",
    doc_dest = "share/doc/nlopt",
    docs = [
        "AUTHORS",
        "NEWS",
    ],
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/nlopt",
    license_docs = glob([
        "**/COPYING",
        "**/COPYRIGHT",
    ]),
    targets = [":nlopt"],
    deps = [":install_cmake_config"],
)

python_lint()
