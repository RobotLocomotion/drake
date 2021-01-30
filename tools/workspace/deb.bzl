# -*- python -*-

load(
    "@drake//tools/workspace:execute.bzl",
    "execute_and_return",
)

def setup_new_deb_archive(repo_ctx):
    """Behaves like new_deb_archive, except that (1) this is a macro instead of
    a rule and (2) this macro returns an error status instead of fail()ing.
    The return value is a struct with a field `error` that will be None on
    success or else a detailed message on any failure.
    """
    name = repo_ctx.attr.name
    filenames = repo_ctx.attr.filenames
    mirrors = repo_ctx.attr.mirrors
    sha256s = repo_ctx.attr.sha256s
    build_file = repo_ctx.attr.build_file

    # Download and unpack all of the debs.
    for i in range(len(filenames)):
        filename = filenames[i]
        if i == len(sha256s):
            sha256s = sha256s + [""]
        sha256 = sha256s[i]
        if not sha256:
            # We do not permit an empty checksum; empty means "don't care".
            sha256 = "0" * 64
        repo_ctx.download(
            url = [mirror + "/" + filename for mirror in mirrors],
            output = filename,
            sha256 = sha256,
        )
        result = execute_and_return(
            repo_ctx,
            ["dpkg-deb", "-x", filename, "."],
        )
        if result.error:
            return result

    # Add in the build file.
    repo_ctx.symlink(build_file, "BUILD.bazel")

    # Success.
    return struct(error = None)

def _impl(repo_ctx):
    result = setup_new_deb_archive(repo_ctx)
    if result.error != None:
        fail("Unable to complete setup for @{} repository: {}".format(
            # (forced line break)
            repo_ctx.name,
            result.error,
        ))

new_deb_archive = repository_rule(
    attrs = {
        "filenames": attr.string_list(
            doc = """
            Base filenames of the debs, e.g., ["libfoo-dev_123_amd64.deb"].
            When multiple files are listed, they will all be extracted atop
            each other (within our sandbox), as is typical for Debian install.
            """,
            mandatory = True,
            allow_empty = False,
        ),
        "mirrors": attr.string_list(
            doc = """
            List of URLs to download from, without the filename portion, e.g.,
            ["https://example.com/archives"].
            """,
            mandatory = True,
            allow_empty = False,
        ),
        "sha256s": attr.string_list(
            doc = """
            Checksums of the files.  When unsure, you may set it to an empty
            string or list; the checksum error will offer a suggestion.  The
            sha256s and filenames are matched ordering (i.e., parallel lists).
            """,
        ),
        "build_file": attr.label(
            doc = """
            Label for BUILD.bazel file to add into the repository.  This should
            contain the rules that expose the archive contents for consumers.
            The *.deb file contents will appear at ".", so paths are like,
            e.g., `hdrs = glob(["usr/include/foo/**/*.h"]),`.
            """,
            mandatory = True,
            allow_files = True,
        ),
    },
    implementation = _impl,
)

"""A repository rule that downloads and unpacks one or more *.deb files.
"""
