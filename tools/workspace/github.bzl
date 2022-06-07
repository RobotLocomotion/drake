# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:utils.bzl", "patch")
load(
    "@drake//tools/workspace:metadata.bzl",
    "generate_repository_metadata",
)

def github_archive(
        name,
        repository = None,
        commit = None,
        commit_pin = None,
        is_branch = False,
        sha256 = "0" * 64,
        build_file = None,
        extra_strip_prefix = "",
        local_repository_override = None,
        mirrors = None,
        **kwargs):
    """A macro to be called in the WORKSPACE that adds an external from GitHub
    using a workspace rule.

    Args:
        name: required name is the rule name and so is used for @name//...
            labels when referring to this archive from BUILD files.
        repository: required GitHub repository name in the form
            organization/project.
        commit: required commit is the tag name, branch name, or git commit sha
            to download; you must set is_branch=True when using a branch name.
        commit_pin: optional boolean, set to True iff the archive should remain
            at the same version indefinitely, eschewing automated upgrades to
            newer versions.
        is_branch: optional boolean, set to True iff the `commit` refers to a
            branch name (not a tag name nor git commit sha). Try to avoid using
            this option; the tip of a branch is generally not robust enough to
            point to a definitive revision and sha256 checksum.
        sha256: required sha256 is the expected SHA-256 checksum of the
            downloaded archive. When unsure, you can omit this argument (or
            comment it out) and then the checksum-mismatch error message will
            offer a suggestion.
        build_file: optional build file is the BUILD file label to use for
            building this external. When omitted, the BUILD file(s) within the
            archive will be used.
        extra_strip_prefix: optional path to strip from the downloaded archive,
            e.g., "src" to root the repository at "./src/" instead of "./".
        local_repository_override: optional local repository override can be
            used for temporary local testing; instead of retrieving the code
            from GitHub, the code is retrieved from the local filesystem path
            given in the argument.
        mirrors: required mirrors is a dict from string to list-of-string with
            key "github", where the list-of-strings are URLs to use, formatted
            using {repository} and {commit} string substitutions. The
            mirrors.bzl file in this directory provides a reasonable default
            value.
    """
    if repository == None:
        fail("Missing repository=")
    if commit == None:
        fail("Missing commit=")
    if mirrors == None:
        fail("Missing mirrors=; see mirrors.bzl")

    if local_repository_override != None:
        path = local_repository_override
        if extra_strip_prefix:
            path += "/" + extra_strip_prefix
        if build_file == None:
            native.local_repository(
                name = name,
                path = path,
            )
        else:
            native.new_local_repository(
                name = name,
                build_file = build_file,
                path = path,
            )
        return

    # Once we've handled the "local_repository_override" sidestep, we delegate
    # to a rule (not a macro) so that we have more leeway in the actions we can
    # take (i.e., so we can do more than just a simple download-and-extract).
    _github_archive_real(
        name = name,
        repository = repository,
        commit = commit,
        commit_pin = commit_pin,
        is_branch = is_branch,
        sha256 = sha256,
        build_file = build_file,
        extra_strip_prefix = extra_strip_prefix,
        mirrors = mirrors,
        **kwargs
    )

# Helper stub to implement a repository_rule in terms of a setup() function.
def _github_archive_real_impl(repository_ctx):
    result = setup_github_repository(repository_ctx)
    if result.error != None:
        fail("Unable to complete setup for " +
             "@{} repository: {}".format(
                 repository_ctx.name,
                 result.error,
             ))

_github_archive_real = repository_rule(
    implementation = _github_archive_real_impl,
    attrs = {
        "repository": attr.string(
            mandatory = True,
        ),
        "commit": attr.string(
            mandatory = True,
        ),
        "commit_pin": attr.bool(),
        "is_branch": attr.bool(),
        "sha256": attr.string(
            mandatory = False,
            default = "0" * 64,
        ),
        "build_file": attr.label(
            default = None,
        ),
        "extra_strip_prefix": attr.string(
            default = "",
        ),
        "mirrors": attr.string_list_dict(
            mandatory = True,
            allow_empty = False,
        ),
        "patches": attr.label_list(
            default = [],
        ),
        "patch_tool": attr.string(
            default = "patch",
        ),
        "patch_args": attr.string_list(
            default = ["-p0"],
        ),
        "patch_cmds": attr.string_list(
            default = [],
        ),
    },
)
"""This is a rule() formulation of the github_archive() macro.  It is identical
to the macro except that it does not support local_repository_override.
Consult the macro documentation for full API details.
"""

def setup_github_repository(repository_ctx):
    """This is a reusable formulation of the github_archive() macro. It is
    identical to the macro except that (1) it does not support local repository
    override, and (2) it returns a status struct instead of failing internally.
    The result struct has a field `error` that will be non-None iff there were
    any errors.  Consult the macro documentation for additional API details.
    """

    # Do the download step first.  (This also writes the metadata.)
    github_download_and_extract(
        repository_ctx,
        repository = repository_ctx.attr.repository,
        commit = repository_ctx.attr.commit,
        commit_pin = getattr(repository_ctx.attr, "commit_pin", None),
        is_branch = getattr(repository_ctx.attr, "is_branch", False),
        mirrors = repository_ctx.attr.mirrors,
        sha256 = repository_ctx.attr.sha256,
        extra_strip_prefix = repository_ctx.attr.extra_strip_prefix,
    )

    # Optionally apply source patches, using Bazel's utility helper.  Here we
    # use getattr as a guard, in case the originating repository_rule didn't
    # want to declare attr support for the patchfile-related settings.
    patch_triggers = ["patches", "patch_cmds"]
    if any([getattr(repository_ctx.attr, a, None) for a in patch_triggers]):
        patch(repository_ctx)

    # We re-implement Bazel's workspace_and_buildfile utility, so that options
    # we don't care about (e.g., build_file_content) do not have to be declared
    # as attrs on our all of our own repository rules.
    repository_ctx.file("WORKSPACE", "workspace(name = \"{name}\")\n".format(
        name = repository_ctx.name,
    ))
    if repository_ctx.attr.build_file:
        for name in ["BUILD", "BUILD.bazel"]:
            if repository_ctx.path(name).exists:
                repository_ctx.execute(["/bin/mv", name, name + ".ignored"])
        repository_ctx.symlink(repository_ctx.attr.build_file, "BUILD.bazel")
    return struct(error = None)

def github_download_and_extract(
        repository_ctx,
        repository,
        commit,
        mirrors,
        output = "",
        sha256 = "0" * 64,
        extra_strip_prefix = "",
        commit_pin = None,
        is_branch = False):
    """Download an archive of the provided GitHub repository and commit to the
    output path and extract it.

    Args:
        repository_ctx: context of a Bazel repository rule.
        repository: GitHub repository name in the form organization/project.
        commit: git revision for which the archive should be downloaded.
        mirrors: dictionary of mirrors, see mirrors.bzl in this directory for
            an example.
        output: path to the directory where the archive will be unpacked,
            relative to the Bazel repository directory.
        sha256: expected SHA-256 hash of the archive downloaded. Fallback to
            an incorrect default value to prevent the hash check from being
            disabled, but allow the first download attempt to fail and print
            the correct SHA-256 hash.
        extra_strip_prefix: optional path to strip from the downloaded archive,
            e.g., "src" to root the repository at "./src/" instead of "./".
        commit_pin: set to True iff the archive should remain at the same
            version indefinitely, eschewing automated upgrades to newer
            versions.
        is_branch: set to True iff the commit refers to a branch name.
    """
    urls = _urls(
        repository = repository,
        commit = commit,
        mirrors = mirrors,
        is_branch = is_branch,
    )

    repository_ctx.download_and_extract(
        urls,
        output = output,
        sha256 = _sha256(sha256),
        type = "tar.gz",
        stripPrefix = _strip_prefix(repository, commit, extra_strip_prefix),
        canonical_id = "redownload_now_5",
    )

    # Create a summary file for Drake maintainers.
    generate_repository_metadata(
        repository_ctx,
        repository_rule_type = "github",
        repository = repository,
        commit = commit,
        version_pin = commit_pin,
        sha256 = sha256,
        urls = urls,
    )

def _sha256(sha256):
    """Fallback to an incorrect default value of SHA-256 hash to prevent the
    hash check from being disabled, but allow the first download attempt of an
    archive to fail and print the correct hash.

    Args:
        sha256: expected SHA-256 hash of the archive to be downloaded.
    """
    if not sha256:
        sha256 = "0" * 64

    return sha256

def _strip_prefix(repository, commit, extra_strip_prefix):
    """Compute the strip prefix for a downloaded archive of the provided
    GitHub repository and commit.

    Args:
        repository: GitHub repository name in the form organization/project.
        commit: git revision for which the archive was downloaded.
    """
    repository_split = repository.split("/")

    if len(repository_split) != 2:
        fail("repository must be formatted as organization/project")

    _, project = repository_split

    # GitHub archives omit the "v" in version tags, for some reason.
    if commit[0] == "v":
        strip_commit = commit[1:]
    else:
        strip_commit = commit

    result = project + "-" + strip_commit.replace("/", "-")
    if extra_strip_prefix:
        result += "/" + extra_strip_prefix
    return result

def _is_commit_sha(commit):
    """Returns true iff the commit is a hexadecimal string of length 40."""
    return len(commit) == 40 and commit.isalnum() and all([
        ch.isdigit() or (ch >= "a" and ch <= "z")
        for ch in commit.elems()
    ])

def _format_url(*, pattern, repository, commit, is_branch):
    """Given a URL pattern for github.com or a Drake-specific mirror,
    substitutes in the given repository and commit (tag or git sha).

    Set is_branch to True iff the commit refers to a branch name.

    The URL pattern accepts the following substitutions:

    The {repository} is always substituted with `repository`.
    The {commit} is always substituted with `commit`.
    If `commit` refers to a git tag, then {tag_name} will be substituted.
    If `commit` refers to a git branch, then {branch_name} will be substituted.
    If `commit` refers to a git sha, then {commit_sha} will be substituted.

    Patterns that contain a substitution which does not apply to the given
    `commit` (e.g., {commit_sha} when `commit` is a tag) will return None.
    The pattern must contain exactly one of {commit}, {tag_name},
    {branch_name}, or {commit_sha}.
    """
    is_commit_sha = _is_commit_sha(commit)
    is_tag = not is_branch and not is_commit_sha
    substitutions = {
        "repository": repository,
        "commit": commit,
        "tag_name": commit if is_tag else None,
        "branch_name": commit if is_branch else None,
        "commit_sha": commit if is_commit_sha else None,
    }
    for name, value in substitutions.items():
        if value == None:
            needle = "{" + name + "}"
            if needle in pattern:
                # If the pattern uses a substitution that we do not have,
                # report that to our caller as "None"; don't return a URL
                # string with a literal "None" in it!
                return None
    return pattern.format(**substitutions)

def _urls(*, repository, commit, mirrors, is_branch):
    """Compute the urls from which an archive of the provided GitHub
    repository and commit may be downloaded.

     Args:
        repository: GitHub repository name in the form organization/project.
        commit: git revision for which the archive should be downloaded.
        mirrors: dictionary of mirrors, see mirrors.bzl in this directory for
            an example.
        is_branch: set to True iff the commit refers to a branch name.
    """
    result_with_nulls = [
        _format_url(
            pattern = x,
            repository = repository,
            commit = commit,
            is_branch = is_branch,
        )
        for x in mirrors.get("github")
    ]
    return [
        url
        for url in result_with_nulls
        if url != None
    ]
