# -*- mode: python -*-
# vi: set ft=python :

"""
Makes selected Boost headers available to be used as a C/C++ dependency.

Example:
    WORKSPACE:
        load("//tools:boost.bzl", "boost_repository")
        boost_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:any"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

HDRS_PATTERNS = [
    "include/boost/{}.h",
    "include/boost/{}.hpp",
    "include/boost/{}_fwd.h",
    "include/boost/{}_fwd.hpp",
    "include/boost/{}/**/*.h",
    "include/boost/{}/**/*.hpp",
    "include/boost/{}/**/*.ipp",
]

LINUX_PREFIX = "/usr"

MAC_PREFIX = "/usr/local/opt/boost"

def _boost_cc_library(os_name, name, hdrs = None, deps = None):
    if os_name == "mac os x":
        prefix = MAC_PREFIX
        srcs = ["empty.cc"]

    else:
        prefix = LINUX_PREFIX
        srcs = []

    if not hdrs:
        hdrs = []

    hdr_paths = []

    for hdr in hdrs:
        hdr_paths += ["{}/{}".format("include", hdr)]

    glob_hdr_paths = [pattern.format(name) for pattern in HDRS_PATTERNS]

    if not deps:
        deps = []

    content = """
cc_library(
    name = "{}",
    srcs = {},
    hdrs = glob({}) + glob({}),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = {},
)
    """.format(name, srcs, hdr_paths, glob_hdr_paths, deps)

    return content

def _impl(repository_ctx):
    if repository_ctx.os.name == "mac os x":
        repository_ctx.file("empty.cc", executable = False)
        prefix = MAC_PREFIX

    elif repository_ctx.os.name == "linux":
        prefix = LINUX_PREFIX

    else:
        fail("Operating system is NOT supported",
             attr = repository_ctx.os.name)

    repository_ctx.symlink("{}/include".format(prefix), "include")

    # Note that we only create library targets for enough of Boost to support
    # those used directly or indirectly by Drake.

    # TODO(jamiesnape): Create a script to help generate the targets.

    file_content = _boost_cc_library(
        repository_ctx.os.name,
        "any",
        deps = [
            ":config",
            ":core",
            ":mpl",
            ":static_assert",
            ":type_index",
            ":type_traits",
            ":utility",
        ],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "assert",
        hdrs = ["boost/current_function.hpp"],
        deps = [":config"],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "blank",
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "compatibility",
        hdrs = ["boost/limits.hpp"],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "config",
        hdrs = ["boost/version.hpp"],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "core",
        hdrs = [
            "boost/checked_delete.hpp",
            "boost/noncopyable.hpp",
            "boost/swap.hpp",
        ]
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "detail",
        deps = [":compatibility"],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "exception",
        hdrs = ["boost/exception_ptr.hpp"],
        deps = [":config"],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "functional",
        deps = [":detail"],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "integer",
        hdrs = [
            "boost/cstdint.hpp",
            "boost/integer_traits.hpp",
        ],
        deps = [":static_assert"],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "math",
        hdrs = ["boost/cstdfloat.hpp"],
        deps = [":integer"],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "move",
        deps = [
            ":assert",
            ":detail",
            ":static_assert",
        ],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "mpl",
        deps = [
            ":move",
            ":preprocessor",
        ],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "preprocessor",
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "static_assert",
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "type_index",
        deps = [
            ":core",
            ":utility",
        ],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "type_traits",
        hdrs = ["boost/aligned_storage.hpp"],
        deps = [
            ":core",
            ":mpl",
            ":static_assert",
        ],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "utility",
        hdrs = [
            "boost/call_traits.hpp",
            "boost/compressed_pair.hpp",
            "boost/cstdlib.hpp",
            "boost/next_prior.hpp",
            "boost/throw_exception.hpp",
        ],
        deps = [
            ":assert",
            ":exception",
        ],
    )

    file_content += _boost_cc_library(
        repository_ctx.os.name,
        "variant",
        deps = [
            ":blank",
            ":config",
            ":detail",
            ":functional",
            ":math",
            ":static_assert",
            ":type_index",
            ":type_traits",
            ":utility",
        ],
    )

    repository_ctx.file("BUILD", content = file_content, executable = False)

boost_repository = repository_rule(implementation = _impl)
