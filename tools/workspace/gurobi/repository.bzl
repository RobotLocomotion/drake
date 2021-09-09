# -*- python -*-
# This is a Bazel repository_rule for the Gurobi solver.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

load("@drake//tools/workspace:os.bzl", "determine_os")

# Ubuntu only: GUROBI_HOME should be the linux64 directory in the Gurobi 9.0.2
# release.
#
def _gurobi_impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        # Gurobi must be installed into its standard location.
        gurobi_home = "/Library/gurobi902/mac64"
        repo_ctx.symlink(gurobi_home, "gurobi-distro")
        build_flavor = "macos"
    else:
        # The default directory for the downloaded gurobi is
        # /opt/gurobi902/linux64. If the user does not use the default
        # directory, the he/she should set GUROBI_HOME environment variable to
        # the gurobi file location.
        gurobi_home = repo_ctx.os.environ.get("GUROBI_HOME", "")
        repo_ctx.symlink(
            gurobi_home or "/opt/gurobi902/linux64",
            "gurobi-distro",
        )
        build_flavor = "ubuntu"

    # Emit the generated BUILD.bazel file.
    repo_ctx.template(
        "BUILD.bazel",
        Label("@drake//tools/workspace/gurobi:" +
              "package-{}.BUILD.bazel.in".format(build_flavor)),
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
