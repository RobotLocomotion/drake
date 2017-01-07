# -*- python -*-

# This file provides a repository rule:
#
#   soft_failure_binary_repository(name, path)
#
# The rule brings an external program into the workspace in a soft-failure way.
# For example, an invocation like this ...
#
#  soft_failure_binary_repository(name = "foo", path = "/usr/local/bin/bar")
#
# ... will create a binary target @foo://bar that can be used via "bazel run",
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
# to hard-code where the binary comes from.  What we *can* do is have a rule in
# our generated BUILD file that yields different results.  For that, we use a
# glob() call, detailed below.
#
# When building the repository, we add a symlink for where the binary should
# be.  By using a glob against that symlink in our BUILD, Bazel will notice at
# build time whether or not the symlink resolves, and bring it into the sandbox
# only if so.  Bazel notices when glob outcomes change, so will rebuild anytime
# the actual binary (dis)appears.  Later at runtime, our wrapper sh_binary will
# see if the globbed file exists, and emit a good message if not.

def _soft_failure_binary_repository_impl(repository_ctx):
    name = repository_ctx.name
    path = getattr(repository_ctx.attr, "path")
    basename = path[(path.rfind("/") + 1):]

    # Symlink the requested binary's path into the repository.  The symlink
    # might be broken, but we only use it at build time via a glob.
    repository_ctx.symlink(path, "soft-failure-binary")

    # Emit the BUILD file into the repository.  Define a sh_binary named as the
    # target binary's basename (so "/path/to/tool" turns into label ":tool").
    # The sh_binary source code will be our wrapper script, defined below.
    warning = (
        "The target @%s//:%s will not run correctly because %s is missing" % (
            name, basename, path))
    BUILD = """
    data = glob(["soft-failure-binary"]) or print("%s");
    sh_binary(
        name = "%s",
        srcs = ["soft-failure-wrapper.sh"],
        data = data,
        visibility = ["//visibility:public"],
    )
    """.replace("\n", " ") % (warning, basename)
    repository_ctx.file("BUILD", content=BUILD)

    # Emit a wrapper that senses whether the glob found anything.  If yes, just
    # delegate to the target program. If not, die with a diagnostic message.
    wrapper_sh = """#!/bin/sh
    wrapped_binary="external/%s/soft-failure-binary"
    if [ ! -x "$wrapped_binary" ]; then
      echo "tools/soft_failure.bzl: as of last build, no such file %s"
      exit 1
    fi
    "$wrapped_binary" "$@"
    """ % (name, path)
    repository_ctx.file("soft-failure-wrapper.sh", content=wrapper_sh)

soft_failure_binary_repository = repository_rule(
    attrs = {
        "path": attr.string(mandatory = True),
    },
    local = True,
    implementation = _soft_failure_binary_repository_impl,
)
