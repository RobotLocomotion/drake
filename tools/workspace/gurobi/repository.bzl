# This is a Bazel repository_rule for the Gurobi solver.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

# Finds the "latest" f'{path}/{prefix}*/{subdir}', where "latest" is determined
# by converting the part that matched the '*' to an integer and taking the
# match with the highest value.
def _find_latest(repo_ctx, path, prefix, subdir):
    best_dir = None
    best_version = None
    for d in repo_ctx.path(path).readdir():
        if d.basename.startswith(prefix):
            full_dir = d.get_child(subdir)
            if full_dir.exists:
                version = int(d.basename[len(prefix):])
                if best_version == None or version > best_version:
                    best_version = version
                    best_dir = str(full_dir)

    return best_dir or (path + "/" + prefix + "-notfound/" + subdir)

# Ubuntu only: GUROBI_HOME should be the linux64 directory in the Gurobi 12.0
# release.
#
def _gurobi_impl(repo_ctx):
    os_name = repo_ctx.os.name
    if os_name == "mac os x":
        os_name = "darwin"

    if os_name == "darwin":
        # Gurobi must be installed into its standard location.
        gurobi_home = _find_latest(
            repo_ctx,
            "/Library",
            "gurobi120",
            "macos_universal2",
        )
        repo_ctx.symlink(gurobi_home, "gurobi-distro")
    elif os_name == "linux":
        # The default directory for the downloaded Gurobi is
        # /opt/gurobi120*/linux64. If the user does not use the default
        # directory, then they should set the GUROBI_HOME environment variable
        # to the Gurobi file location.
        gurobi_home = repo_ctx.getenv("GUROBI_HOME", "")
        repo_ctx.symlink(
            gurobi_home or _find_latest(
                repo_ctx,
                "/opt",
                "gurobi120",
                "linux64",
            ),
            "gurobi-distro",
        )
    else:
        # Defer error reporting to the BUILD file.
        repo_ctx.symlink("/gurobi-notfound", "gurobi-distro")
        os_name = "linux"

    # Emit the generated BUILD.bazel file.
    repo_ctx.template(
        "BUILD.bazel",
        Label("@drake//tools/workspace/gurobi:" +
              "package-{}.BUILD.bazel.in".format(os_name)),
        substitutions = {
            "{gurobi_home}": gurobi_home,
        },
        executable = False,
    )

    # Capture whether or not Gurobi tests can run in parallel.
    license_unlimited_int = repo_ctx.getenv("DRAKE_GUROBI_LICENSE_UNLIMITED", "0")  # noqa: E501
    license_unlimited = bool(int(license_unlimited_int) == 1)
    repo_ctx.file(
        "defs.bzl",
        content = "DRAKE_GUROBI_LICENSE_UNLIMITED = {}"
            .format(license_unlimited),
        executable = False,
    )

gurobi_repository = repository_rule(
    local = True,
    implementation = _gurobi_impl,
)
