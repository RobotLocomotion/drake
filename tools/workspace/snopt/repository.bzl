# -*- python -*-

"""
Either extracts an gzipped tar archive containing SNOPT source code located at
SNOPT_PATH or if SNOPT_PATH is "git", clones a remote git repository
containing SNOPT source code, checks out the specified branch, commit, or tag
and makes the targets available for binding.

If either branch or tag are provided, then the rule will determine and return
a dict containing the values of commit and shallow_since that may be
substituted to provide a reproducible version of this rule.

Arguments:
    name: A unique name for this rule.
    remote: Location of the remote git repository.
    commit: Commit in the remote git repository to be checked out. At most one
        of branch, commit, or tag must be provided.
    shallow_since: Optional date, not after the specified commit. Not allowed
        if a branch or tag is specified. Provide a unix timestamp
        corresponding to the date of commit to avoid generating a debug
        message.
    tag: Tag in the remote git repository to be checked out. At most one of
        branch, commit, or tag must be provided.
    branch: Branch in the remote git repository to be checked out. At most one
        of branch, commit, or tag must be provided.
"""

load(
    "@bazel_tools//tools/build_defs/repo:utils.bzl",
    "patch",
    "update_attrs",
)
load(
    "@bazel_tools//tools/build_defs/repo:git_worker.bzl",
    "git_repo",
)
load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)
load(
    "@drake//tools/workspace:execute.bzl",
    "execute_and_return",
)

def snopt_repository(
        name,
        remote = "git@github.com:RobotLocomotion/snopt.git",
        commit = None,
        shallow_since = None,
        tag = None,
        branch = None):
    if not branch and not commit and not tag:
        commit = "0254e961cb8c60193b0862a0428fd6a42bfb5243"
        shallow_since = "1546539374 -0500"

    _snopt_repository(
        name = name,
        remote = remote,
        commit = commit,
        shallow_since = shallow_since,
        tag = tag,
        branch = branch,
    )

def _setup_git(repo_ctx):
    # Download the snopt sources from an access-controlled git repository.
    # We'll use git operations directly (and not the github_archive helper)
    # because github_archive does not (yet, easily) support authentication.
    #
    # If a user wishes to use a different protocol that ssh for the remote,
    # note that we will respect their config file, so they can configure
    # the protocol without changing the attributes of the repository_rule.
    # For example:
    # git config --global url.https://github.com/.insteadOf git@github.com:
    #
    # Based on https://github.com/bazelbuild/bazel/blob/0.28.0/tools/build_defs/repo/git.bzl#L24-L55. # noqa
    if (not repo_ctx.attr.branch and not repo_ctx.attr.commit and not repo_ctx.attr.tag) or (repo_ctx.attr.commit and repo_ctx.attr.tag) or (repo_ctx.attr.branch and repo_ctx.attr.tag) or (repo_ctx.attr.branch and repo_ctx.attr.commit):  # noqa
        fail("Exactly one of branch, commit, or tag must be provided")

    git_repo_info = git_repo(repo_ctx, str(repo_ctx.path(".")))
    repo_ctx.symlink(repo_ctx.attr.build_file, "BUILD.bazel")
    patch(repo_ctx)
    repo_ctx.delete(repo_ctx.path(".git"))

    # Note that in Bazel 0.28, you cannot specify *by commit* a commit that
    # exists not on a branch, but only as tag, therefore we silence the debug
    # message in the case of tags. Note also that specifying a shallow_since
    # value in a format other than a unix timestamp will generate a debug
    # message even if it is equivalent to the unix timestamp of the given
    # commit.
    if repo_ctx.attr.tag:
        return None

    attrs_to_update = {
        "commit": git_repo_info.commit,
        "shallow_since": git_repo_info.shallow_since,
    }

    updated_attrs = update_attrs(
        repo_ctx.attr,
        _attrs.keys(),
        attrs_to_update,
    )

    # If we found the actual commit, remove all other means of specifying
    # it.
    if "commit" in updated_attrs:
        updated_attrs.pop("tag", None)
        updated_attrs.pop("branch", None)

    return updated_attrs

def _extract_local_archive(repo_ctx, snopt_path):
    # TODO(jwnimmer-tri) Perhaps in the future we should allow SNOPT_PATH
    # to also refer to the *.zip format of the download, and/or an already-
    # unpacked source archive directory.
    if not (snopt_path.startswith("/") and snopt_path.endswith(".tar.gz")):
        return "SNOPT_PATH of '{}' is malformed".format(snopt_path)
    if not repo_ctx.path(snopt_path).exists:
        return "SNOPT_PATH of '{}' does not exist".format(snopt_path)

    repo_ctx.report_progress("Extracting archive")

    result = execute_and_return(repo_ctx, [
        "tar",
        "--gunzip",
        "--extract",
        "--file",
        repo_ctx.path(snopt_path).realpath,
        "--strip-components=1",
    ])
    if result.error:
        return result.error

    patch(repo_ctx)
    return None

def _setup_deferred_failure(repo_ctx, error_message):
    # Produce a repository with a valid BUILD.bazel file, but where all of the
    # targets emit an error_message at build-time (but not while loading).
    repo_ctx.file(
        "error.txt",
        "ERROR: Repository rule @{} failed: {}\n".format(
            repo_ctx.name,
            error_message,
        ),
    )
    repo_ctx.symlink(
        Label("@drake//tools/workspace/snopt:package-error.BUILD.bazel"),
        "BUILD.bazel",
    )

def _setup_local_archive(repo_ctx, snopt_path):
    error = _extract_local_archive(repo_ctx, snopt_path)
    if error == None:
        repo_ctx.symlink(repo_ctx.attr.build_file, "BUILD.bazel")
    else:
        _setup_deferred_failure(repo_ctx, error)

def _impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)

    updated_attrs = None
    snopt_path = repo_ctx.os.environ.get("SNOPT_PATH", "")

    if len(snopt_path) == 0:
        # When SNOPT is enabled (e.g., with `--config snopt`), then SNOPT_PATH
        # must be set.  If it's not set, we'll defer the error messages to the
        # build phase, instead of loading phase.  This deferment enables
        # `genquery()` calls that reference `@snopt` to succeed, even if SNOPT
        # is disabled and we don't have access to its source code.
        #
        # Once the user sets a SNOPT_PATH, this function will be re-run
        # (because we tag `environ` on our repository_rule).  In this way, we
        # can keep this rule tagged `local = False`, which is important for not
        # re-running git anytime the dependency graph changes.
        _setup_deferred_failure(
            repo_ctx,
            "SNOPT was enabled via '--config snopt' or '--config everything'" +
            " (possibly in a '.bazelrc' file) but the SNOPT_PATH environment" +
            " variable is unset.",
        )
    elif snopt_path == "git":
        # This case does not use deferred error handling.  If you set
        # SNOPT_PATH=git, then we'll assume you know what you're doing,
        # and that the git operations should always succeed.
        updated_attrs = _setup_git(repo_ctx)
    else:
        # This case uses deferred error handling, since doing so is easy.
        _setup_local_archive(repo_ctx, snopt_path)

    # Add in the helper.
    if os_result.is_ubuntu or os_result.is_manylinux:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/snopt:fortran-ubuntu.bzl"),
            "fortran.bzl",
        )
    elif os_result.is_macos:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/snopt:fortran-macos.bzl"),
            "fortran.bzl",
        )
    else:
        fail("Operating system is NOT supported {}".format(os_result))

    return updated_attrs

_attrs = {
    "remote": attr.string(),
    "commit": attr.string(),
    "shallow_since": attr.string(),
    "tag": attr.string(),
    "branch": attr.string(),
    "init_submodules": attr.bool(),
    "verbose": attr.bool(),
    "patches": attr.label_list(
        default = ["@drake//tools/workspace/snopt:snopt-openmp.patch"],
    ),
    "patch_cmds": attr.string_list(),
    "patch_tool": attr.string(default = "patch"),
    "patch_args": attr.string_list(default = ["-p0"]),
    "build_file": attr.label(
        allow_single_file = True,
        default = "@drake//tools/workspace/snopt:package.BUILD.bazel",
    ),
    "recursive_init_submodules": attr.bool(),
}

_snopt_repository = repository_rule(
    attrs = _attrs,
    environ = [
        "BAZEL_SH",
        "SNOPT_PATH",
    ],
    local = False,
    implementation = _impl,
)
