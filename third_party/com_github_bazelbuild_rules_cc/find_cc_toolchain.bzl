load("@rules_cc//cc/common:cc_common.bzl", "cc_common")

# This function is forked and modified from bazelbuild/rules_cc as of:
# https://github.com/bazelbuild/rules_cc/blob/262ebec/cc/find_cc_toolchain.bzl
def find_cc_toolchain(ctx):
    # Check the incompatible flag for toolchain resolution.
    if hasattr(cc_common, "is_cc_toolchain_resolution_enabled_do_not_use") and cc_common.is_cc_toolchain_resolution_enabled_do_not_use(ctx = ctx):  # noqa
        valid_names = [
            # The name for Bazel 6 and earlier.
            "//cc:toolchain_type",
            # The name for Bazel 7 and after.
            "@@bazel_tools//tools/cpp:toolchain_type",
        ]
        for possible_name in valid_names:
            if possible_name in ctx.toolchains:
                info = ctx.toolchains[possible_name]
                if all([
                    hasattr(info, x)
                    for x in ["cc_provider_in_toolchain", "cc"]
                ]):
                    # This logic is cherry-picked from upstream d5d830b.
                    return info.cc
                return info
        fail("In order to use find_cc_toolchain, your rule has to depend on C++ toolchain. See find_cc_toolchain.bzl docs for details.")  # noqa

    # Fall back to the legacy implicit attribute lookup.
    if hasattr(ctx.attr, "_cc_toolchain"):
        return ctx.attr._cc_toolchain[cc_common.CcToolchainInfo]

    # We didn't find anything.
    fail("In order to use find_cc_toolchain, your rule has to depend on C++ toolchain. See find_cc_toolchain.bzl docs for details.")  # noqa
