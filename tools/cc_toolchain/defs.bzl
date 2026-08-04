load("@rules_cc//cc:action_names.bzl", "ACTION_NAMES")
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain")
load("//tools/skylark:cc.bzl", "CcInfo", "cc_common")

def _generate_cmake_bazelrc_impl(ctx):
    # Retrieve the link_flags from the current C++ toolchain.
    cc_toolchain = find_cc_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
    )
    link_variables = cc_common.create_link_variables(
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        is_linking_dynamic_library = True,
    )
    link_flags = cc_common.get_memory_inefficient_command_line(
        feature_configuration = feature_configuration,
        action_name = ACTION_NAMES.cpp_link_dynamic_library,
        variables = link_variables,
    )

    # If a multi-threaded linker is being used, we need to put it back into
    # single-threaded mode to avoid overloading the machine.
    content = ""
    if "-fuse-ld=lld" in link_flags or "-fuse-ld=mold" in link_flags:
        content = "build --linkopt=-Wl,--threads=1\n"

    # Write the bazelrc fragment.
    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(out, content, is_executable = False)
    return [
        DefaultInfo(
            files = depset([out]),
            data_runfiles = ctx.runfiles(files = [out]),
        ),
    ]

generate_cmake_bazelrc = rule(
    implementation = _generate_cmake_bazelrc_impl,
    doc = """
Generates a (possibly-empty) bazelrc fragment that contains recommended flags
when building Drake with our CMakeLists.txt wrapper. In particular, checks if
the linker is multi-threaded and if so sets a thread limit.
    """,
    attrs = {
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    fragments = ["cpp"],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
)
