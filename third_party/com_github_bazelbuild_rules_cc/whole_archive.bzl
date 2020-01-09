# -*- python -*-

# Copyright 2019 The Bazel Authors. All rights reserved.
# Copyright 2019 Toyota Research Institute. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This function is forked and modified from bazelbuild/rules_cc as of:
# https://github.com/bazelbuild/rules_cc/blob/262ebec3c2296296526740db4aefce68c80de7fa/cc/find_cc_toolchain.bzl
def _find_cc_toolchain(ctx):
    # Check the incompatible flag for toolchain resolution.
    if hasattr(cc_common, "is_cc_toolchain_resolution_enabled_do_not_use") and cc_common.is_cc_toolchain_resolution_enabled_do_not_use(ctx = ctx):  # noqa
        if "//cc:toolchain_type" in ctx.toolchains:
            return ctx.toolchains["//cc:toolchain_type"]
        fail("In order to use find_cc_toolchain, your rule has to depend on C++ toolchain. See find_cc_toolchain.bzl docs for details.")  # noqa

    # Fall back to the legacy implicit attribute lookup.
    if hasattr(ctx.attr, "_cc_toolchain"):
        return ctx.attr._cc_toolchain[cc_common.CcToolchainInfo]

    # We didn't find anything.
    fail("In order to use find_cc_toolchain, your rule has to depend on C++ toolchain. See find_cc_toolchain.bzl docs for details.")  # noqa

# This function is forked and modified from bazelbuild/rules_cc as of:
# https://github.com/bazelbuild/rules_cc/blob/262ebec3c2296296526740db4aefce68c80de7fa/examples/my_c_archive/my_c_archive.bzl
def _cc_whole_archive_library_impl(ctx):
    # Find the C++ toolchain.
    cc_toolchain = _find_cc_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    # Iterate over the transitive list of libraries we want to link, adding
    # `alwayslink = True` to each one.
    deps_cc_infos = cc_common.merge_cc_infos(
        cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    )
    old_libraries_to_link = deps_cc_infos.linking_context.libraries_to_link.to_list()  # noqa
    new_libraries_to_link = []
    for old_library_to_link in old_libraries_to_link:
        new_library_to_link = cc_common.create_library_to_link(
            actions = ctx.actions,
            feature_configuration = feature_configuration,
            cc_toolchain = cc_toolchain,
            static_library = old_library_to_link.static_library,
            pic_static_library = old_library_to_link.pic_static_library,
            dynamic_library = old_library_to_link.resolved_symlink_dynamic_library,  # noqa
            interface_library = old_library_to_link.resolved_symlink_interface_library,  # noqa
            # This is where the magic happens!
            alwayslink = True,
        )
        new_libraries_to_link.append(new_library_to_link)

    # Return the CcInfo to pass along to code that wants to link us.
    linking_context = cc_common.create_linking_context(
        libraries_to_link = new_libraries_to_link,
        user_link_flags = deps_cc_infos.linking_context.user_link_flags,
        additional_inputs = deps_cc_infos.linking_context.additional_inputs.to_list(),  # noqa
    )
    return [
        DefaultInfo(
            runfiles = ctx.runfiles(
                collect_data = True,
                collect_default = True,
            ),
        ),
        CcInfo(
            compilation_context = deps_cc_infos.compilation_context,
            linking_context = linking_context,
        ),
    ]

# Forked and modified from bazelbuild/rules_cc as of:
# https://github.com/bazelbuild/rules_cc/blob/262ebec3c2296296526740db4aefce68c80de7fa/examples/my_c_archive/my_c_archive.bzl
cc_whole_archive_library = rule(
    implementation = _cc_whole_archive_library_impl,
    attrs = {
        "deps": attr.label_list(providers = [CcInfo]),
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    fragments = ["cpp"],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
)
"""Creates an cc_library with `alwayslink = True` added to all of its deps, to
work around https://github.com/bazelbuild/bazel/issues/7362 not providing any
useful way to create shared libraries from multiple cc_library targets unless
you want even statically-linked programs to keep all of their symbols.
"""
