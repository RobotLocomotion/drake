# -*- python -*-

def github_archive(
        name,
        repository = None,
        commit = None,
        sha256 = None,
        build_file = None,
        local_repository_override = None,
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

    The optional local_repository_override= can be used for temporary local
    testing; instead of retrieving the code from github, the code is retrieved
    from the local filesystem path given in the argument.
    """
    if repository == None:
        fail("Missing repository=")
    if commit == None:
        fail("Missing commit=")
    if sha256 == None or len(sha256) == 0:
        # This is mostly-required, but we fallback to a wrong-default value to
        # allow the first attempt to fail and print the correct sha256.
        sha256 = "0" * 64

    # Packages are mirrored from GitHub to CloudFront backed by an S3 bucket.
    mirrors = [
        "https://github.com/%s/archive/%s.tar.gz",
        "https://drake-mirror.csail.mit.edu/github/%s/%s.tar.gz",
        "https://s3.amazonaws.com/drake-mirror/github/%s/%s.tar.gz",
    ]
    urls = [mirror % (repository, commit) for mirror in mirrors]

    repository_split = repository.split("/")
    if len(repository_split) != 2:
        fail("The repository= must be formatted as 'organization/project'")
    _, project = repository_split
    strip_commit = commit
    if commit[0] == 'v':
        # Github archives omit the "v" in version tags, for some reason.
        strip_commit = commit[1:]
    strip_prefix = project + "-" + strip_commit

    if local_repository_override != None:
        if build_file == None:
            native.local_repository(
                name = name,
                path = local_repository_override)
        else:
            native.new_local_repository(
                name = name,
                build_file = build_file,
                path = local_repository_override)
        return

    if build_file == None:
        native.http_archive(
            name = name,
            urls = urls,
            sha256 = sha256,
            strip_prefix = strip_prefix,
            **kwargs)
    else:
        native.new_http_archive(
            name = name,
            urls = urls,
            sha256 = sha256,
            build_file = build_file,
            strip_prefix = strip_prefix,
            **kwargs)
