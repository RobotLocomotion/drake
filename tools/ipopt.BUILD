# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)
load("@drake//tools:python_lint.bzl", "python_lint")

package(default_visibility = ["//visibility:public"])

# We build IPOPT by shelling out to autotools.

# We run autotools in a genrule, and only files explicitly identified as
# outputs of that genrule can be made available to other rules. Therefore, we
# need a list of every file in the IPOPT install.
# See https://github.com/bazelbuild/bazel/issues/281.

# find include/coin -name "*.h" -o -name "*.hpp" -o -name "*.hdd" | sort |
# sed 's/$/",/g'| sed 's/^/"/g'
IPOPT_HDRS = [
    "include/coin/AmplTNLP.hpp",
    "include/coin/HSLLoader.h",
    "include/coin/IpAlgTypes.hpp",
    "include/coin/IpBlas.hpp",
    "include/coin/IpCachedResults.hpp",
    "include/coin/IpCompoundVector.hpp",
    "include/coin/IpDebug.hpp",
    "include/coin/IpDenseVector.hpp",
    "include/coin/IpException.hpp",
    "include/coin/IpExpansionMatrix.hpp",
    "include/coin/IpIpoptApplication.hpp",
    "include/coin/IpIpoptCalculatedQuantities.hpp",
    "include/coin/IpIpoptData.hpp",
    "include/coin/IpIpoptNLP.hpp",
    "include/coin/IpIteratesVector.hpp",
    "include/coin/IpJournalist.hpp",
    "include/coin/IpLapack.hpp",
    "include/coin/IpMatrix.hpp",
    "include/coin/IpNLP.hpp",
    "include/coin/IpNLPScaling.hpp",
    "include/coin/IpObserver.hpp",
    "include/coin/IpoptConfig.h",
    "include/coin/IpOptionsList.hpp",
    "include/coin/IpOrigIpoptNLP.hpp",
    "include/coin/IpReferenced.hpp",
    "include/coin/IpRegOptions.hpp",
    "include/coin/IpReturnCodes.h",
    "include/coin/IpReturnCodes.hpp",
    "include/coin/IpReturnCodes_inc.h",
    "include/coin/IpSmartPtr.hpp",
    "include/coin/IpSolveStatistics.hpp",
    "include/coin/IpStdCInterface.h",
    "include/coin/IpSymMatrix.hpp",
    "include/coin/IpTaggedObject.hpp",
    "include/coin/IpTimedTask.hpp",
    "include/coin/IpTimingStatistics.hpp",
    "include/coin/IpTNLPAdapter.hpp",
    "include/coin/IpTNLP.hpp",
    "include/coin/IpTNLPReducer.hpp",
    "include/coin/IpTypes.hpp",
    "include/coin/IpUtils.hpp",
    "include/coin/IpVector.hpp",
    "include/coin/PardisoLoader.h",
    "include/coin/ThirdParty/arith.h",
    "include/coin/ThirdParty/asl.h",
    "include/coin/ThirdParty/asl_pfg.h",
    "include/coin/ThirdParty/asl_pfgh.h",
    "include/coin/ThirdParty/defs.h",
    "include/coin/ThirdParty/dmumps_c.h",
    "include/coin/ThirdParty/funcadd.h",
    "include/coin/ThirdParty/getstub.h",
    "include/coin/ThirdParty/macros.h",
    "include/coin/ThirdParty/metis.h",
    "include/coin/ThirdParty/mpi.h",
    "include/coin/ThirdParty/mumps_compat.h",
    "include/coin/ThirdParty/mumps_c_types.h",
    "include/coin/ThirdParty/nlp2.h",
    "include/coin/ThirdParty/nlp.h",
    "include/coin/ThirdParty/proto.h",
    "include/coin/ThirdParty/psinfo.h",
    "include/coin/ThirdParty/rename.h",
    "include/coin/ThirdParty/stdio1.h",
    "include/coin/ThirdParty/struct.h",
]

# ls lib | grep "\.a$" | sed 's/$/",/g'| sed 's/^/"lib\//g'
# These are artisanally topo-sorted: demand before supply.
# If you change the order, you may get undefined-reference linker errors.
IPOPT_LIBS = [
    "lib/libipopt.a",
    "lib/libipoptamplinterface.a",
    "lib/libcoinmumps.a",
    # TODO(#4913): Remove the dependency on METIS.
    "lib/libcoinmetis.a",
    "lib/libcoinasl.a",
    "lib/libcoinlapack.a",
    "lib/libcoinblas.a",
]

# Invokes ./configure, make, and make install to build IPOPT. We arbitrarily
# allow make to use a number of cores equal to half the number of available
# cores, rounded up.
#
# We emit static libraries because dynamic libraries would have different names
# on OS X and on Linux, and Bazel genrules don't allow platform-dependent outs.
# https://github.com/bazelbuild/bazel/issues/281
genrule(
    name = "build_with_autotools",
    srcs = glob(["**/*"]),
    outs = IPOPT_HDRS + IPOPT_LIBS,
    cmd = " ".join([
        "(",
        "env",
        "cdexec=$(location @kythe//tools/cdexec:cdexec)",
        "top_builddir=$(@D)",
        "$(location @drake//tools:ipopt_build_with_autotools.sh)",
        " 2>&1 > ipopt_build_with_autotools.log",
        ")",
        "|| (cat ipopt_build_with_autotools.log && false)",
    ]),
    tools = [
        "@drake//tools:ipopt_build_with_autotools.sh",
        "@kythe//tools/cdexec:cdexec",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "ipopt",
    srcs = IPOPT_LIBS,
    hdrs = IPOPT_HDRS,
    includes = ["include/coin"],
    linkstatic = 1,
    deps = ["@gfortran"],
    alwayslink = 1,
)

cmake_config(
    package = "IPOPT",
    script = "@drake//tools:ipopt-create-cps.py",
    version_file = "Ipopt/src/Common/config_ipopt_default.h",
)

install_cmake_config(package = "IPOPT")  # Creates rule :install_cmake_config.

# TODO(jamiesnape): At the moment libipopt.a has gone AWOL.
install(
    name = "install",
    doc_dest = "share/doc/ipopt",
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/ipopt",
    hdr_strip_prefix = ["include/coin"],
    license_docs = glob(["**/LICENSE"]),
    targets = [":ipopt"],
    deps = [":install_cmake_config"],
)

python_lint()
