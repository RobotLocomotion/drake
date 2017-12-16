# -*- python -*-

def _execute(repository_ctx, command):
    """Run the command (list); fail on non-zero return code."""
    result = repository_ctx.execute(command)
    if result.return_code != 0:
        message = "Failure running " + (
            " ".join(["'{}'".format(x) for x in command]))
        if result.stdout:
            message += "\n" + result.stdout
        if result.stderr:
            message += "\n" + result.stderr
        fail(message)
    return result

def _execute_which(repository_ctx, command):
    """Run the command (list); fail on non-zero return code.  The first element
    of command is replaced by $(which command[0]), failing if not found.
    """
    found = repository_ctx.which(command[0])
    if not found:
        fail("Could not find a program named '{}'".format(command[0]))
    return _execute(repository_ctx, [found] + command[1:])

def _impl(repository_ctx):
    name = repository_ctx.attr.name
    filenames = repository_ctx.attr.filenames
    mirrors = repository_ctx.attr.mirrors
    sha256s = repository_ctx.attr.sha256s
    build_file = repository_ctx.attr.build_file

    # Download and unpack all of the debs.
    for i in range(len(filenames)):
        filename = filenames[i]
        if i < len(sha256s):
            sha256 = sha256s[i]
        else:
            sha256 = ""
        if not sha256:
            # We do not permit an empty checksum; empty means "don't care".
            sha256s[i] = "0" * 64
        repository_ctx.download(
            url = [mirror + "/" + filename for mirror in mirrors],
            output = filename,
            sha256 = sha256,
        )
        _execute_which(repository_ctx, ["dpkg-deb", "-x", filename, "."])

    repository_ctx.symlink(build_file, "BUILD.bazel")

"""A repository rule that download and unpacks one or more *.deb files.
"""

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
            single_file = True,
            allow_files = True,
        ),
    },
    implementation = _impl,
)
