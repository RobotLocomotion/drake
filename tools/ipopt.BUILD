# We build IPOPT by shelling out to autotools.

# A prefix-string for genrule cmd attributes, which uses the Kythe cdexec tool,
# in quiet mode, to execute in the genrule output directory.
CDEXEC = "$(location @//tools/third_party/kythe/tools/cdexec:cdexec) -q $(@D)"

# We run autotools in a genrule, and only files explicitly identified as outputs
# of that genrule can be made available to other rules. Therefore, we need a
# list of every file in the IPOPT install.
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

# ls lib | grep lib | grep -v "la$" | sed 's/$/",/g'| sed 's/^/"lib\//g'
IPOPT_LIBS_LINUX = [
  "lib/libcoinasl.so",
  "lib/libcoinasl.so.0",
  "lib/libcoinasl.so.0.0.0",
  "lib/libcoinblas.so",
  "lib/libcoinblas.so.0",
  "lib/libcoinblas.so.0.0.0",
  "lib/libcoinlapack.so",
  "lib/libcoinlapack.so.0",
  "lib/libcoinlapack.so.0.0.0",
  "lib/libcoinmetis.so",
  "lib/libcoinmetis.so.0",
  "lib/libcoinmetis.so.0.0.0",
  "lib/libcoinmumps.so",
  "lib/libcoinmumps.so.0",
  "lib/libcoinmumps.so.0.0.0",
  "lib/libipoptamplinterface.so",
  "lib/libipoptamplinterface.so.0",
  "lib/libipoptamplinterface.so.0.0.0",
  "lib/libipopt.so",
  "lib/libipopt.so.0",
  "lib/libipopt.so.0.0.0",
]

# ls lib | grep "dylib$" | sed 's/$/",/g' | sed 's/^/"lib\//g'
IPOPT_LIBS_APPLE = [
  "lib/libcoinasl.0.0.0.dylib",
  "lib/libcoinasl.0.dylib",
  "lib/libcoinasl.dylib",
  "lib/libcoinblas.0.0.0.dylib",
  "lib/libcoinblas.0.dylib",
  "lib/libcoinblas.dylib",
  "lib/libcoinlapack.0.0.0.dylib",
  "lib/libcoinlapack.0.dylib",
  "lib/libcoinlapack.dylib",
  "lib/libcoinmetis.0.0.0.dylib",
  "lib/libcoinmetis.0.dylib",
  "lib/libcoinmetis.dylib",
  "lib/libcoinmumps.0.0.0.dylib",
  "lib/libcoinmumps.0.dylib",
  "lib/libcoinmumps.dylib",
  "lib/libipopt.0.0.0.dylib",
  "lib/libipopt.0.dylib",
  "lib/libipopt.dylib",
  "lib/libipoptamplinterface.0.0.0.dylib",
  "lib/libipoptamplinterface.0.dylib",
  "lib/libipoptamplinterface.dylib",
]

# Invokes ./configure, make, and make install to build IPOPT. We arbitrarily
# use make -j 8 and hope for the best in terms of overall CPU consumption, since
# Bazel has no way to tell a genrule how many cores it should use.

# We also have to create separate targets for headers and object code, because
# the header files have the same names on all platforms, but the object code
# extensions vary.  This is wasteful, but rebuilds should be rare.

BUILD_IPOPT_CMD = (
    CDEXEC + " `pwd`/external/ipopt/configure 2> /dev/null" + " && " +
    CDEXEC + " make -j 8 2> /dev/null" + " && " +
    CDEXEC + " make install 2> /dev/null")

genrule(
    name = "build_headers_with_autotools",
    srcs = glob(["*", "**/*"]),
    outs = IPOPT_HDRS,
    tools = ["@//tools/third_party/kythe/tools/cdexec:cdexec"],
    cmd = BUILD_IPOPT_CMD,
    visibility = ["//visibility:private"],
)

genrule(
    name = "build_linux_with_autotools",
    srcs = glob(["*", "**/*"]),
    outs = IPOPT_LIBS_LINUX,
    tools = ["@//tools/third_party/kythe/tools/cdexec:cdexec"],
    cmd = BUILD_IPOPT_CMD,
    visibility = ["//visibility:private"],
)

genrule(
    name = "build_apple_with_autotools",
    srcs = glob(["*", "**/*"]),
    outs = IPOPT_LIBS_APPLE,
    tools = ["@//tools/third_party/kythe/tools/cdexec:cdexec"],
    cmd = BUILD_IPOPT_CMD,
    visibility = ["//visibility:private"],
)

cc_library(
    name = "ipopt",
    hdrs = IPOPT_HDRS,
    srcs = select({
        "@//tools:linux": IPOPT_LIBS_LINUX,
        "@//tools:apple": IPOPT_LIBS_APPLE,
    }),
    visibility = ["//visibility:public"],
    includes = ["include/coin"],
)