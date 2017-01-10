# -*- python -*-

# This file provides a repository rule:
#
#   soft_failure_binary_repository(name, local_path)
#
# The rule brings an external program into the workspace in a soft-failure way.
# For example, an invocation like this ...
#
#  soft_failure_binary_repository(
#    name = "foo",
#    local_path = "/usr/local/bin/bar"
#  )
#
# ... will create a binary target @foo//:bar that can be used via "bazel run",
# or as in the data=[...] attribute of some other rule, test, etc.  At build
# time the path will be symlinked in the repository; the build will succeed
# even if the referenced path is missing.  Only if the @foo//:bar target is
# actually run and the referenced path does not exist will we die with a
# diagnostic.  The pass / failure condition is hermetic; if the program later
# (dis)appears, any targets that depended on *running* it will be rebuilt.
#
# === Implementation notes follow ==
#
# This rule is tricky due to https://github.com/bazelbuild/bazel/issues/1595.
# In the repository rule, we can't do any sensing that will vary over time
# (such as $PATH lookup or "$(which ...)"), so instead we must ask our caller
# to hard-code where the program comes from.  What we can do is have a rule in
# our generated BUILD file that yields different results.  For that, we use a
# glob() call, detailed below.
#
# When building the repository, we add a symlink for where the program should
# be.  By using glob() against that symlink in our BUILD, Bazel will notice at
# build time whether or not the symlink resolves, and bring it into the sandbox
# only if so.  Bazel notices when glob() outcomes change, so rebuilds anytime
# the actual program (dis)appears.

def _soft_failure_binary_repository_impl(repository_ctx):
    repo_name = repository_ctx.name
    local_path = getattr(repository_ctx.attr, "local_path")
    basename = local_path[(local_path.rfind("/") + 1):]

    # Symlink the requested binary's path into the repository.  The symlink
    # might be broken, but we are careful to handle that below.
    symlink_basename = "__symlink_" + basename
    repository_ctx.symlink(local_path, symlink_basename)
    symlink_path = "external/" + repo_name + "/" + symlink_basename

    # Set up an error message to be used by generated code.
    label = "@{repo_name}//:{basename}".format(
        repo_name=repo_name, basename=basename)
    warning = "soft_failure.bzl: " + (
        "{label} does not work because {local_path} was missing".format(
            label=label, local_path=local_path))

    # Emit the wrapper that unconditionally fails at runtime; this is selected
    # at build-time when the glob() fails.
    wrapper_failure_basename = "__failure_" + basename
    wrapper_sh = """#!/bin/sh
    echo "{warning}"
    exit 1
    """.format(warning=warning)
    repository_ctx.file(wrapper_failure_basename, content=wrapper_sh)

    # Emit the wrapper that delegates to the target program; this is selected
    # at build-time when the glob() succeeds.
    wrapper_success_basename = "__success_" + basename
    wrapper_sh = """#!/bin/sh
    if [ ! -L "{symlink_path}" ]; then
      # We are running outside of the sandbox.
      exec "{local_path}" "$@"
    fi
    if [ ! -x "{symlink_path}" ]; then
      # We are running inside of the sandbox; the symlink exists but is broken.
      echo "{warning}; this is unexpected, because it was present at build-time"
      exit 1
    fi
    exec "{symlink_path}" "$@"
    """.format(
        symlink_path=symlink_path,
        local_path=local_path,
        warning=warning)
    repository_ctx.file(wrapper_success_basename, content=wrapper_sh)

    # Emit the BUILD file into the repository.  Define a sh_binary named as the
    # target binary's basename (so "/path/to/tool" turns into label ":tool").
    # The sh_binary source code will be one of the two wrapper scripts above.
    BUILD = """
    data = glob(["{symlink_basename}"]) or print("{warning}")
    src = "{wrapper_success_basename}" if data else "{wrapper_failure_basename}"
    sh_binary(
        name = "{basename}",
        srcs = [src],
        data = data,
        visibility = ["//visibility:public"],
    )
    """.format(
        symlink_basename=symlink_basename,
        warning=warning,
        basename=basename,
        wrapper_success_basename=wrapper_success_basename,
        wrapper_failure_basename=wrapper_failure_basename)
    BUILD = BUILD.replace("\n    ", "\n")  # Strip leading indent from lines.
    repository_ctx.file("BUILD", content=BUILD)

soft_failure_binary_repository = repository_rule(
    attrs = {
        "local_path": attr.string(mandatory = True),
    },
    local = True,
    implementation = _soft_failure_binary_repository_impl,
)
