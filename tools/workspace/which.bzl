# -*- python -*-

def _impl(repository_ctx):
    command = repository_ctx.attr.command
    found_command = repository_ctx.which(command)
    if not found_command:
        fail("Could not find {} on PATH={}".format(
            command, repository_ctx.os.environ["PATH"]))
    repository_ctx.symlink(found_command, command)
    build_file_content = """
# A symlink to {}.
exports_files(["{}"])
""".format(found_command, command)
    repository_ctx.file(
        "BUILD", content = build_file_content, executable = False)

which = repository_rule(
    attrs = {
        "command": attr.string(mandatory = True),
    },
    environ = ["PATH"],
    local = True,
    implementation = _impl,
)

"""Alias the result of $(which $command) into a BUILD label @$name//:$command
(or @$command if name and command match).

Changes to $PATH will cause this rule to be re-evaluated (because it sets its
environ attribute), and changes to any WORKSPACE or BUILD file will also cause
this rule to be re-evaluated (because it sets its local attribute).

However, note that if none of $PATH, WORKSPACE, nor **/BUILD.bazel change, then
this rule will not be re-evaluated.  This means that adding or removing the
presence of `command` on some entry in $PATH will not be accounted for until
something else changes.  To force an update; run bazel with $PATH set to some
temporarily-different value to trigger a new search.

Args:
    command (:obj:`str`): Short name of command, e.g., "cat".

"""
