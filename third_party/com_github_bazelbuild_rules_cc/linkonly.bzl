# -*- python -*-

# Copyright 2019 The Bazel Authors. All rights reserved.
# Copyright 2021 Toyota Research Institute. All rights reserved.
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

def _cc_linkonly_library_impl(ctx):
    deps_cc_infos = cc_common.merge_cc_infos(
        cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    )
    return [
        DefaultInfo(
            runfiles = ctx.runfiles(
                collect_data = True,
                collect_default = True,
            ),
        ),
        CcInfo(
            compilation_context = None,
            linking_context = deps_cc_infos.linking_context,
        ),
    ]

cc_linkonly_library = rule(
    implementation = _cc_linkonly_library_impl,
    attrs = {
        "deps": attr.label_list(providers = [CcInfo]),
    },
    fragments = ["cpp"],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
)
"""Links the given dependencies but discards the entire compilation context,
i.e., include paths and preprocessor definitions.
"""
