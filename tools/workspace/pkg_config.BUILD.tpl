# -*- bazel -*-

# %{topcomment}

load("@drake//tools/skylark:cc.bzl", "cc_library")

licenses(%{licenses})

package(default_visibility = ["//visibility:public"])

cc_library(
    name = %{library_name},
    srcs = %{srcs},
    hdrs = %{hdrs},
    copts = %{copts},
    defines = %{defines},
    includes = %{includes},
    isystem = True,
    linkopts = %{linkopts},
    deps = %{deps},
    deprecation = %{extra_deprecation},
)

%{build_epilog}
