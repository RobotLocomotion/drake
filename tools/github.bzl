# -*- python -*-

def github_archive(
        name,
        repository = None,
        commit = None,
        sha256 = None,
        build_file = None,
        **kwargs):
    """A macro to be called in the WORKSPACE that adds an external from github
    using a workspace rule.

    The required name= is the rule name and so is used for @name//... labels
    when referring to this archive from BUILD files.

    The required commit= is the git hash to download.  (When the git project is
    also a git submodule in CMake, this should be kept in sync with the git
    submodule commit used there.)  This can also be a tag.

    The required sha256= is the checksum of the downloaded archive.  When
    unsure, you can omit this argument (or comment it out) and then the
    checksum-mismatch error message message will offer a suggestion.

    The optional build_file= is the BUILD file label to use for building this
    external.  When omitted, the BUILD file(s) within the archive will be used.

    """
    if repository == None:
        fail("Missing repository=")
    if commit == None:
        fail("Missing commit=")
    if sha256 == None:
        # This is mostly-required, but we fallback to a wrong-default value to
        # allow the first attempt to fail and print the correct sha256.
        sha256 = "0" * 64

    urls = [
        "https://github.com/%s/archive/%s.tar.gz" % (repository, commit)
    ]

    repository_split = repository.split("/")
    if len(repository_split) != 2:
        fail("The repository= must be formatted as 'organization/project'")
    _, project = repository_split
    strip_commit = commit
    if commit[0] == 'v':
        # Github archives omit the "v" in version tags, for some reason.
        strip_commit = commit[1:]
    strip_prefix = project + "-" + strip_commit

    if build_file == None:
        native.http_archive(
            name=name,
            urls=urls,
            sha256=sha256,
            strip_prefix=strip_prefix,
            **kwargs)
    else:
        native.new_http_archive(
            name=name,
            urls=urls,
            sha256=sha256,
            build_file=build_file,
            strip_prefix=strip_prefix,
            **kwargs)
