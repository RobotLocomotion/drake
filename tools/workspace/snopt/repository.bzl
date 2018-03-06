# -*- python -*-

def _execute(repo_ctx, mnemonic, *command):
    # Run the command, with fail() a non-zero returncode.
    result = repo_ctx.execute(*command)
    if result.return_code:
        fail("Repository rule @{} error {} during {} operation: {}".format(
            repo_ctx.name, result.return_code, mnemonic,
            repr(result.stdout + result.stderr),
        ))

def _run_git(repo_ctx, *args):
    # Runs the git operation named in *args.  We use "--git-dir" to prevent git
    # from looking in parent directories to find the repository.  We only ever
    # want it to look where we've told it.
    _execute(repo_ctx, "git", ["git", "--git-dir=.git"] + list(args))

def _git_clone(
        repo_ctx,
        remote = None,
        commit = None):
    # Clones a git repository named by the given remote and reset it to the
    # given commit.
    #
    # The clone is placed into "." (which is the workspace being populated by
    # whatever repository_rule calls this function).  This function assumes
    # that "." is empty, which that's already been established by Bazel before
    # it invokes the repository_rule.
    #
    # On any error, terminates via fail() and does not return.
    #
    # We would prefer to load `@bazel_tools//tools/build_defs/repo:git.bzl` and
    # use its `git_repository` rule, but there is no flavor of that rule that
    # allows us to pass in the repo_ctx from our own repository_rule.
    (commit and remote) or fail("Missing commit or remote")
    _run_git(repo_ctx, "clone", remote, ".")
    _run_git(repo_ctx, "reset", "--hard", commit)

def _impl(repo_ctx):
    snopt_path = repo_ctx.os.environ.get("SNOPT_PATH", "")
    if not snopt_path:
        # Download the snopt sources from an access-controlled git repository.
        # We'll use git operations directly (and not the github_archive helper)
        # because github_archive does not (yet, easily) support authentication.
        #
        # If a user wishes to use a different protocol that ssh for the remote,
        # note that we will respect their config file, so they can configure
        # the protocol without changing the attributes of the repository_rule.
        # For example:
        # git config --global url.https://github.com/.insteadOf git@github.com:
        _git_clone(
            repo_ctx,
            remote = repo_ctx.attr.remote,
            commit = repo_ctx.attr.commit,
        )
    else:
        # TODO(jwnimmer-tri) Perhaps in the future we should allow SNOPT_PATH
        # to also refer to the *.zip format of the download, and/or an already-
        # unpacked source archive directory.
        if not (snopt_path.startswith("/") and snopt_path.endswith(".tar.gz")):
            fail("SNOPT_PATH of '{}' is malformed".format(snopt_path))
        if not repo_ctx.path(snopt_path).exists:
            fail("SNOPT_PATH of '{}' does not exist".format(snopt_path))
        _execute(repo_ctx, "tar", [
            "tar", "--gunzip", "--extract",
            "--file", repo_ctx.path(snopt_path).realpath,
            "--strip-components=1",
        ])
    if repo_ctx.attr.use_drake_build_rules:
        # Disable any files that came from the upstream snopt source archive.
        _execute(repo_ctx, "find-and-mv", ["bash", "-c", """
            set -euxo pipefail
            find . -name BUILD -print0 -o -name BUILD.bazel -print0 |
                xargs -t -n1 -0 -I{} \
                mv {} {}.upstream-ignored
        """])
        # Link Drake's BUILD file into the snopt workspace.
        repo_ctx.symlink(
            Label("@drake//tools/workspace/snopt:package.BUILD.bazel"),
            "BUILD")

snopt_repository = repository_rule(
    attrs = {
        "remote": attr.string(default = "git@github.com:RobotLocomotion/snopt.git"),  # noqa
        "commit": attr.string(default = "c17db3769e59d4a8d651631d5d79641cecca0504"),  # noqa
        "use_drake_build_rules": attr.bool(default = True),
    },
    environ = ["SNOPT_PATH"],
    local = False,
    implementation = _impl,
)
