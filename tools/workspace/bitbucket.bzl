# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:utils.bzl", "patch")
load(
    "@drake//tools/workspace:metadata.bzl",
    "generate_repository_metadata",
)

# The github.bzl and bitbucket.bzl implementations are very similar.  Try to
# keep the two files relatively well-synchronized -- or even better, rework
# them to share code where doing so is an improvement.

def bitbucket_archive(
        name,
        repository = None,
        commit = None,
        sha256 = "0" * 64,
        strip_prefix = None,
        build_file = None,
        mirrors = None,
        **kwargs):
    """A macro to be called in the WORKSPACE that adds an external from
    bitbucket using a workspace rule.

    Args:
        name: required name is the rule name and so is used for @name//...
            labels when referring to this archive from BUILD files.
        repository: required Bitbucket repository name.
        commit: required commit is the hash to download. (When a git project
            is also a git submodule in CMake, this should be kept in sync with
            the git submodule commit used there.) This can also be a tag.
        sha256: required sha256 is the expected SHA-256 checksum of the
            downloaded archive. When unsure, you can omit this argument (or
            comment it out) and then the checksum-mismatch error message will
            offer a suggestion.
        strip_prefix: required strip prefix is the directory prefix to strip
            from the extracted files.
        build_file: optional build file is the BUILD file label to use for
            building this external. When omitted, the BUILD file(s) within the
            archive will be used.
        mirrors: required mirrors is a dict from string to list-of-string with
            key "bitbucket", where the list-of-strings are URLs to use,
            formatted using {repository} and {commit} string substitutions.
            The mirrors.bzl file in this directory provides a reasonable
            default value.
    """
    if repository == None:
        fail("Missing repository=")
    if commit == None:
        fail("Missing commit=")
    if strip_prefix == None:
        fail("Missing strip_prefix=")
    if mirrors == None:
        fail("Missing mirrors=; see mirrors.bzl")

    repository_split = repository.split("/")
    if len(repository_split) != 2:
        fail("The repository= must be formatted as 'organization/project'")

    # TODO(jwnimmer-tri) We should support local_repository_override here.

    # We delegate to a rule (not a macro) here so that we have more leeway in
    # the actions we can take (i.e., so we can do more than just a simple
    # download-and-extract).
    _bitbucket_archive_real(
        name = name,
        repository = repository,
        commit = commit,
        sha256 = sha256,
        strip_prefix = strip_prefix,
        build_file = build_file,
        mirrors = mirrors,
        **kwargs
    )

# Helper stub to implement a repository_rule in terms of a setup() function.
def _bitbucket_archive_real_impl(repository_ctx):
    result = setup_bitbucket_repository(repository_ctx)
    if result.error != None:
        fail("Unable to complete setup for " +
             "@{} repository: {}".format(
                 repository_ctx.name,
                 result.error,
             ))

_bitbucket_archive_real = repository_rule(
    implementation = _bitbucket_archive_real_impl,
    attrs = {
        "repository": attr.string(
            mandatory = True,
        ),
        "commit": attr.string(
            mandatory = True,
        ),
        "sha256": attr.string(
            mandatory = False,
            default = "0" * 64,
        ),
        "strip_prefix": attr.string(
            mandatory = True,
        ),
        "build_file": attr.label(
            default = None,
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
"""This is a rule() formulation of the bitbucket_archive() macro.  It is
identical to the macro, but in rule form.  Consult the macro documentation for
full API details.
"""

def setup_bitbucket_repository(repository_ctx):
    """This is reusable formulation of the bitbucket_archive() macro.  It is
    identical to the macro except that it returns a status struct, instead of
    failing internally.  The result struct has a field `error` that will be
    non-None iff there were any errors.  Consult the macro documentation for
    additional API details.
    """

    # Do the download step first.  (This also writes the metadata.)
    bitbucket_download_and_extract(
        repository_ctx,
        repository = repository_ctx.attr.repository,
        commit = repository_ctx.attr.commit,
        mirrors = repository_ctx.attr.mirrors,
        sha256 = repository_ctx.attr.sha256,
        strip_prefix = repository_ctx.attr.strip_prefix,
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

def bitbucket_download_and_extract(
        repository_ctx,
        repository,
        commit,
        mirrors,
        output = "",
        sha256 = "0" * 64,
        strip_prefix = ""):
    """Download an archive of the provided Bitbucket repository and commit to
    the output path and extract it.

    Args:
        repository_ctx: context of a Bazel repository rule.
        repository: Bitbucket repository name.
        commit: revision for which the archive should be downloaded.
        mirrors: dictionary of mirrors, see mirrors.bzl in this directory for
            an example.
        output: path to the directory where the archive will be unpacked,
            relative to the Bazel repository directory.
        sha256: expected SHA-256 hash of the archive downloaded. Fallback to
            an incorrect default value to prevent the hash check from being
            disabled, but allow the first download attempt to fail and print
            the correct SHA-256 hash.
    """
    urls = _urls(repository, commit, mirrors)

    repository_ctx.download_and_extract(
        urls,
        output = output,
        sha256 = _sha256(sha256),
        stripPrefix = strip_prefix,
    )

    # Create a summary file for for Drake maintainers.
    generate_repository_metadata(
        repository_ctx,
        repository_rule_type = "bitbucket",
        repository = repository,
        commit = commit,
        sha256 = sha256,
        strip_prefix = strip_prefix,
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

def _urls(repository, commit, mirrors):
    """Compute the urls from which an archive of the provided Bitbucket
    repository and commit may be downloaded.

     Args:
        repository: Bitbucket repository name.
        commit: revision for which the archive should be downloaded.
        mirrors: dictionary of mirrors, see mirrors.bzl in this directory for
            an example.
    """
    return [
        x.format(
            repository = repository,
            commit = commit,
        )
        for x in mirrors.get("bitbucket")
    ]
