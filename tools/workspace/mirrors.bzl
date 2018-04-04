# -*- mode: python -*-
# vi: set ft=python :

# This constant contains Drake's default lists of mirrors.  It is keyed by the
# repository type using magic strings ("github", "bitbucket", etc.), and has
# values of type list-of-string; each string is a pattern for a mirror URL.
#
# When calling a Drake workspace rule that requires a mirror= argument, this
# constant is a reasonable default value.
#
# Each repository type has its own keyword string substitutions within its
# pattern string; these will vary from one repository type to another; consult
# the specific rules (e.g., github_archive()) for details.
#
# The first item in each list is the authoritative source (e.g., the upstream
# server), if there is one.
#
# For Drake's defaults, Packages are mirrored from upstream (GitHub, Bitbucket,
# PyPI, etc.) to CloudFront backed by an S3 bucket.
#
DEFAULT_MIRRORS = {
    "bitbucket": [
        "https://bitbucket.org/{repository}/get/{commit}.tar.gz",
        "https://drake-mirror.csail.mit.edu/bitbucket/{repository}/{commit}.tar.gz",  # noqa
        "https://s3.amazonaws.com/drake-mirror/bitbucket/{repository}/{commit}.tar.gz",  # noqa
    ],
    "buildifier": [
        "https://github.com/bazelbuild/buildtools/releases/download/{version}/{filename}",  # noqa
        "https://drake-mirror.csail.mit.edu/github/bazelbuild/buildtools/releases/{version}/{filename}",  # noqa
        "https://s3.amazonaws.com/drake-mirror/github/bazelbuild/buildtools/releases/{version}/{filename}",  # noqa
    ],
    "director": [
        "https://drake-packages.csail.mit.edu/director/{archive}",
        "https://s3.amazonaws.com/drake-packages/director/{archive}",
    ],
    "github": [
        "https://github.com/{repository}/archive/{commit}.tar.gz",
        "https://drake-mirror.csail.mit.edu/github/{repository}/{commit}.tar.gz",  # noqa
        "https://s3.amazonaws.com/drake-mirror/github/{repository}/{commit}.tar.gz",  # noqa
    ],
    "maven": [
        "https://jcenter.bintray.com/{fulljar}",
        "https://repo1.maven.org/maven2/{fulljar}",
        # N.B. ibiblio doesn't offer https.
        "http://maven.ibiblio.org/maven2/{fulljar}",
    ],
    "pypi": [
        "https://files.pythonhosted.org/packages/source/{p}/{package}/{package}-{version}.tar.gz",  # noqa
        "https://drake-mirror.csail.mit.edu/pypi/{package}/{package}-{version}.tar.gz",  # noqa
        "https://s3.amazonaws.com/drake-mirror/pypi/{package}/{package}-{version}.tar.gz",  # noqa
    ],
    "vtk": [
        "https://drake-packages.csail.mit.edu/vtk/{archive}",
        "https://s3.amazonaws.com/drake-packages/vtk/{archive}",
    ],
}
