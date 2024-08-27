load(
    "@drake//tools/workspace:execute.bzl",
    "execute_or_fail",
)

def _impl(repo_ctx):
    # Add the BUILD file.
    repo_ctx.symlink(
        Label("//tools/workspace/venv:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    # Symlink some paths that we need.
    venv_dir = repo_ctx.path(Label("@drake//:bazel-venv/venv.drake")).realpath
    repo_ctx.symlink("{}/bin".format(venv_dir), "bin")
    repo_ctx.symlink(
        "{}/lib/python3/site-packages".format(venv_dir),
        "site-packages",
    )

    # Run pip-sync to ensure the venv content matches the requirements.txt; it
    # will (un)install any packages as necessary.
    sync = repo_ctx.path(Label("@drake//tools/workspace/venv:sync")).realpath
    repo_ctx.report_progress("Running pip-sync")
    execute_or_fail(repo_ctx, [sync])

    # Tell Bazel to re-run this rule when `sync` or `requirements.txt` change.
    repo_ctx.watch(sync)
    requirements = repo_ctx.path(
        Label("@drake//:bazel-venv/requirements.txt")).realpath
    repo_ctx.watch(requirements)

venv_repository = repository_rule(
    local = False,  # Don't force a re-fetch when the Bazel server restarts.
    implementation = _impl,
)
