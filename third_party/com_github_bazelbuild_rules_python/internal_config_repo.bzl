# Copyright 2023 The Bazel Authors. All rights reserved.
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
"""Repository to generate configuration settings info from the environment.

This handles settings that can't be encoded as regular build configuration flags,
such as globals available to Bazel versions, or propagating user environment
settings for rules to later use.
"""

_ENABLE_PYSTAR_ENVVAR_NAME = "RULES_PYTHON_ENABLE_PYSTAR"
_ENABLE_PYSTAR_DEFAULT = "1"

_CONFIG_TEMPLATE = """\
config = struct(
  enable_pystar = {enable_pystar},
)
"""

# The py_internal symbol is only accessible from within @rules_python, so we have to
# load it from there and re-export it so that rules_python can later load it.
_PY_INTERNAL_SHIM = """\
load("@rules_python//tools/build_defs/python/private:py_internal_renamed.bzl", "py_internal_renamed")
py_internal_impl = py_internal_renamed
"""

ROOT_BUILD_TEMPLATE = """\
load("@bazel_skylib//:bzl_library.bzl", "bzl_library")

package(
    default_visibility = [
        "{visibility}",
    ]
)

bzl_library(
    name = "rules_python_config_bzl",
    srcs = ["rules_python_config.bzl"]
)

bzl_library(
    name = "py_internal_bzl",
    srcs = ["py_internal.bzl"],
    deps = [{py_internal_dep}],
)
"""

def _internal_config_repo_impl(rctx):
    pystar_requested = _bool_from_environ(rctx, _ENABLE_PYSTAR_ENVVAR_NAME, _ENABLE_PYSTAR_DEFAULT)

    # Bazel 7+ (dev and later) has native.starlark_doc_extract, and thus the
    # py_internal global, which are necessary for the pystar implementation.
    if pystar_requested and hasattr(native, "starlark_doc_extract"):
        enable_pystar = pystar_requested
    else:
        enable_pystar = False

    rctx.file("rules_python_config.bzl", _CONFIG_TEMPLATE.format(
        enable_pystar = enable_pystar,
    ))

    if enable_pystar:
        shim_content = _PY_INTERNAL_SHIM
        py_internal_dep = '"@rules_python//tools/build_defs/python/private:py_internal_renamed_bzl"'
    else:
        shim_content = "py_internal_impl = None\n"
        py_internal_dep = ""

    # Bazel 5 doesn't support repository visibility, so just use public
    # as a stand-in
    if native.bazel_version.startswith("5."):
        visibility = "//visibility:public"
    else:
        visibility = "@rules_python//:__subpackages__"

    rctx.file("BUILD", ROOT_BUILD_TEMPLATE.format(
        py_internal_dep = py_internal_dep,
        visibility = visibility,
    ))
    rctx.file("py_internal.bzl", shim_content)
    return None

internal_config_repo = repository_rule(
    implementation = _internal_config_repo_impl,
    environ = [_ENABLE_PYSTAR_ENVVAR_NAME],
)

def _bool_from_environ(rctx, key, default):
    return bool(int(rctx.os.environ.get(key, default)))
