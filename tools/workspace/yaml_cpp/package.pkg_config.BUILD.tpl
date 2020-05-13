# -*- python -*-

# %{topcomment}

licenses(%{licenses})

config_setting(
    name = "linux_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
)

cc_library(
    name = %{name},
    srcs = %{srcs},
    hdrs = %{hdrs},
    copts = %{copts},
    defines = %{defines},
    includes = %{includes},
    linkopts = %{linkopts} + select({
        ":linux": ["-L/usr/lib/x86_64-linux-gnu"],
        "//conditions:default": [],
    }),
    deps = %{deps},
)

%{build_epilog}
