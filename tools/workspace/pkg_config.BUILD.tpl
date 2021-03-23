# -*- python -*-

# %{topcomment}

licenses(%{licenses})

package(default_visibility = ["//visibility:public"])

cc_library(
    name = %{name},
    srcs = %{srcs},
    hdrs = %{hdrs},
    copts = %{copts},
    defines = %{defines},
    includes = %{includes},
    linkopts = %{linkopts},
    deps = %{deps},
    deprecation = %{extra_deprecation},
)

%{build_epilog}
