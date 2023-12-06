load("@bazel_tools//tools/build_defs/repo:utils.bzl", "patch")
load("//tools/workspace:metadata.bzl", "generate_repository_metadata")

def github_archive(
        name,
        repository = None,
        commit = None,
        commit_pin = None,
        sha256 = "0" * 64,
        build_file = None,
        patches = None,
        extra_strip_prefix = "",
        local_repository_override = None,
        mirrors = None,
        upgrade_advice = "",
        **kwargs):
    """A macro to be called in the WORKSPACE that adds an external from GitHub
    using a workspace rule.

    Args:
        name: required name is the rule name and so is used for @name//...
            labels when referring to this archive from BUILD files.
        repository: required GitHub repository name in the form
            organization/project.
        commit: required commit is the tag name or git commit sha to download.
        commit_pin: optional boolean, set to True iff the archive should remain
            at the same version indefinitely, eschewing automated upgrades to
            newer versions.
        sha256: required sha256 is the expected SHA-256 checksum of the
            downloaded archive. When unsure, you can omit this argument (or
            comment it out) and then the checksum-mismatch error message will
            offer a suggestion.
        build_file: optional build file is the BUILD file label to use for
            building this external. As a Drake-specific abbreviation, when
            provided as a relative label (e.g., ":package.BUILD.bazel"), it
            will be taken as relative to the "@drake//tools/workspace/{name}/"
            package. When no build_file is provided, the BUILD file(s) within
            the archive will be used.
        patches: optional list of patches to apply, matching what's described
            at https://bazel.build/rules/lib/repo/git#git_repository-patches.
            As a Drake-specific abbreviation, when provided using relative
            labels (e.g., ":patches/foo.patch"), they will be taken as relative
            to the "@drake//tools/workspace/{name}/" package.
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
        upgrade_advice: optional string that describes extra steps that should
            be taken when upgrading to a new version.
            Used by //tools/workspace:new_release.
    """
    _github_real(
        name,
        repository = repository,
        commit = commit,
        commit_pin = commit_pin,
        sha256 = sha256,
        build_file = build_file,
        patches = patches,
        extra_strip_prefix = extra_strip_prefix,
        local_repository_override = local_repository_override,
        mirrors = mirrors,
        upgrade_advice = upgrade_advice,
        implementation = _github_archive_real_impl,
        attrs = {
            "sha256": attr.string(
                mandatory = False,
                default = "0" * 64,
            ),
        },
        **kwargs
    )

def github_release(
        name,
        implementation = None,
        repository = None,
        commit = None,
        commit_pin = None,
        sha256s = {},
        build_file = None,
        patches = None,
        extra_strip_prefix = "",
        local_repository_override = None,
        mirrors = None,
        upgrade_advice = "",
        **kwargs):
    """A macro to be called in the WORKSPACE that adds an external from GitHub
    using a workspace rule.

    Args:
        name: required name is the rule name and so is used for @name//...
            labels when referring to this archive from BUILD files.
        implementation: required function which sets up the repository.
        repository: required GitHub repository name in the form
            organization/project.
        commit: required commit is the tag name or git commit sha to download.
        commit_pin: optional boolean, set to True iff the archive should remain
            at the same version indefinitely, eschewing automated upgrades to
            newer versions.
        sha256s: required sha256s is a dictionary of the expected SHA-256
            checksums of all candidate downloaded release artifacts. When
            unsure, you can omit this argument (or comment it out) and then the
            checksum-mismatch error message will offer a suggestion.
        build_file: optional build file is the BUILD file label to use for
            building this external. As a Drake-specific abbreviation, when
            provided as a relative label (e.g., ":package.BUILD.bazel"), it
            will be taken as relative to the "@drake//tools/workspace/{name}/"
            package. When no build_file is provided, the BUILD file(s) within
            the archive will be used.
        patches: optional list of patches to apply, matching what's described
            at https://bazel.build/rules/lib/repo/git#git_repository-patches.
            As a Drake-specific abbreviation, when provided using relative
            labels (e.g., ":patches/foo.patch"), they will be taken as relative
            to the "@drake//tools/workspace/{name}/" package.
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
        upgrade_advice: optional string that describes extra steps that should
            be taken when upgrading to a new version.
            Used by //tools/workspace:new_release.
    """
    if implementation == None:
        fail("Missing implementation=")

    _github_real(
        name,
        repository = repository,
        commit = commit,
        commit_pin = commit_pin,
        sha256s = sha256s,
        build_file = build_file,
        patches = patches,
        extra_strip_prefix = extra_strip_prefix,
        local_repository_override = local_repository_override,
        mirrors = mirrors,
        upgrade_advice = upgrade_advice,
        implementation = implementation,
        attrs = {
            "sha256s": attr.string_dict(
                mandatory = False,
                default = {},
            ),
        },
        **kwargs
    )

def _github_real(
        name,
        implementation,
        attrs,
        repository,
        commit,
        build_file,
        patches,
        extra_strip_prefix,
        local_repository_override,
        mirrors,
        **kwargs):
    if repository == None:
        fail("Missing repository=")
    if commit == None:
        fail("Missing commit=")
    if mirrors == None:
        fail("Missing mirrors=; see mirrors.bzl")

    build_file = _resolve_drake_abbreviation(name, build_file)
    patches = [
        _resolve_drake_abbreviation(name, one_patch)
        for one_patch in (patches or [])
    ]

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
    attrs.update({
        "repository": attr.string(
            mandatory = True,
        ),
        "commit": attr.string(
            mandatory = True,
        ),
        "commit_pin": attr.bool(),
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
        "upgrade_advice": attr.string(
            default = "",
        ),
    })
    repository_rule(implementation = implementation, attrs = attrs)(
        name = name,
        repository = repository,
        commit = commit,
        build_file = build_file,
        patches = patches,
        extra_strip_prefix = extra_strip_prefix,
        mirrors = mirrors,
        **kwargs
    )

def _resolve_drake_abbreviation(name, label_str):
    """De-abbreviates the given label_str as a Drake tools/workspace label.
    If the label_str is None, returns None. If the label_str is relative,
    interprets it relative to the "@drake//tools/workspace/{name}/" package
    and returns an absolute label. Otherwise, returns the label_str unchanged.
    """
    if label_str == None:
        return None
    if label_str.startswith(":"):
        return "@drake//tools/workspace/" + name + label_str
    return label_str

# Helper stub to implement a repository_rule in terms of a setup() function.
def _github_archive_real_impl(repository_ctx):
    result = setup_github_repository(repository_ctx)
    if result.error != None:
        fail("Unable to complete setup for " +
             "@{} repository: {}".format(
                 repository_ctx.name,
                 result.error,
             ))

def _download_github_archive(
        repository_ctx,
        repository,
        commit,
        mirrors,
        strip_prefix):
    """Download a github archive."""

    urls = github_format_urls(
        repository = repository,
        commit = commit,
        mirrors = mirrors,
    )
    sha256 = repository_ctx.attr.sha256

    repository_ctx.download_and_extract(
        urls,
        sha256 = _sha256(sha256),
        type = "tar.gz",
        stripPrefix = strip_prefix,
    )
    return sha256, urls

def setup_github_repository(
        repository_ctx,
        downloader = _download_github_archive,
        repository_type = "github"):
    """This is a reusable formulation of the github_archive() macro. It is
    identical to the macro except that (1) it does not support local repository
    override, (2) it allows specifying how to download the artifact and which
    key of mirrors to use, and (3) it returns a status struct instead of
    failing internally.  The result struct has a field `error` that will be
    non-None iff there were any errors.  Consult the macro documentation for
    additional API details.

    Args:
        repository_ctx: context of a Bazel repository rule.
        downloader: optional function to download the appropriate artifact and
            return the resolved checksum and urls.  _download_github_archive()
            is used by default.
            an example.
        repository_type: specifies the repository type, which (among other
            things) determines which entry in the mirror map is used.
    """

    mirrors = repository_ctx.attr.mirrors.get(repository_type)

    repository = repository_ctx.attr.repository
    commit = repository_ctx.attr.commit
    commit_pin = getattr(repository_ctx.attr, "commit_pin", None)
    strip_prefix = _strip_prefix(
        repository,
        commit,
        repository_ctx.attr.extra_strip_prefix,
    )

    # Do the download step first.
    sha256, urls = downloader(
        repository_ctx,
        repository = repository,
        commit = commit,
        mirrors = mirrors,
        strip_prefix = strip_prefix,
    )

    # Create a summary file for Drake maintainers.
    upgrade_advice = getattr(repository_ctx.attr, "upgrade_advice", "")
    upgrade_advice = "\n".join(
        [line.strip() for line in upgrade_advice.strip().split("\n")],
    )

    generate_repository_metadata(
        repository_ctx,
        repository_rule_type = repository_type,
        repository = repository,
        commit = commit,
        version_pin = commit_pin,
        sha256 = sha256,
        urls = urls,
        strip_prefix = strip_prefix,
        upgrade_advice = upgrade_advice,
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
    #
    # Unlike workspace_and_buildfile, we create WORKSPACE.bazel and BUILD.bazel
    # (rather than WORKSPACE and BUILD) because when the "*.bazel" flavor is
    # present, it always takes precedence.
    files_to_be_created = ["WORKSPACE.bazel"]
    if repository_ctx.attr.build_file:
        files_to_be_created.append("BUILD.bazel")
    for name in files_to_be_created:
        if repository_ctx.path(name).exists:
            repository_ctx.execute(["/bin/mv", name, name + ".ignored"])
    repository_ctx.file(
        "WORKSPACE.bazel",
        "workspace(name = \"{name}\")\n".format(
            name = repository_ctx.name,
        ),
    )
    if repository_ctx.attr.build_file:
        repository_ctx.symlink(repository_ctx.attr.build_file, "BUILD.bazel")
    return struct(error = None)

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
    return len(commit) == 40 and all([
        ch.isdigit() or (ch >= "a" and ch <= "f")
        for ch in commit.elems()
    ])

def _format_url(*, pattern, repository, commit, substitutions):
    """Given a URL pattern for github.com or a Drake-specific mirror,
    substitutes in the given repository and commit (tag or git sha).

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
    is_tag = not is_commit_sha
    substitutions = dict(substitutions)
    substitutions.update({
        "repository": repository,
        "commit": commit,
        "tag_name": commit if is_tag else None,
        "commit_sha": commit if is_commit_sha else None,
    })
    for name, value in substitutions.items():
        if value == None:
            needle = "{" + name + "}"
            if needle in pattern:
                # If the pattern uses a substitution that we do not have,
                # report that to our caller as "None"; don't return a URL
                # string with a literal "None" in it!
                return None
    return pattern.format(**substitutions)

def github_format_urls(*, repository, commit, mirrors, substitutions = {}):
    """Compute the urls from which an archive of the provided GitHub
    repository and commit may be downloaded.

     Args:
        repository: GitHub repository name in the form organization/project.
        commit: git revision for which the archive should be downloaded.
        mirrors: list of mirrors, see "github" entry in mirrors.bzl in this
            directory for an example.
    """
    result_with_nulls = [
        _format_url(
            pattern = x,
            repository = repository,
            commit = commit,
            substitutions = substitutions,
        )
        for x in mirrors
    ]
    return [
        url
        for url in result_with_nulls
        if url != None
    ]
