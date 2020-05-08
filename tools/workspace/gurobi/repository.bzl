# -*- python -*-
# This is a Bazel repository_rule for the Gurobi solver.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

load("@drake//tools/workspace:os.bzl", "determine_os")

# Ubuntu only: GUROBI_HOME should be the linux64 directory in the Gurobi 9.0.2
# release.
#
# TODO(jwnimmer-tri) The Gurobi docs use /opt/gurobi902/linux64, so we should
# probably look in that location as a reasonable default guess.
def _gurobi_impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        # Gurobi must be installed into its standard location.
        gurobi_home = "/Library/gurobi902/mac64"
        repo_ctx.symlink(gurobi_home, "gurobi-distro")
    else:
        # Locate Gurobi using an environment variable.  If GUROBI_HOME is
        # unset, pass the empty string to our template() call, but symlink a
        # dummy distro path since we can't symlink to the empty string.
        gurobi_home = repo_ctx.os.environ.get("GUROBI_HOME", "")
        repo_ctx.symlink(gurobi_home or "/no-gurobi-path", "gurobi-distro")

    # Emit the generated BUILD.bazel file.
    repo_ctx.template(
        "BUILD.bazel",
        Label("@drake//tools/workspace/gurobi:" +
              "package-{}.BUILD.bazel.in".format(os_result.distribution)),
        substitutions = {
            "{gurobi_home}": gurobi_home,
        },
        executable = False,
    )

gurobi_repository = repository_rule(
    environ = ["GUROBI_HOME"],
    local = True,
    implementation = _gurobi_impl,
)
