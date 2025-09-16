# This constant contains Drake's default lists of mirrors.  It is keyed by the
# repository type using magic strings ("github", etc.), and has values of type
# list-of-string; each string is a pattern for a mirror URL.
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
    "crate_universe": [
        # This pattern instructs us to allow the crates.io URL.
        "{default_url}",
        # These patterns are made available as additional backups.
        "https://drake-mirror.csail.mit.edu/crates.io/{archive}",
        "https://s3.amazonaws.com/drake-mirror/crates.io/{archive}",
    ],
    "doxygen": [
        "https://drake-mirror.csail.mit.edu/other/doxygen/{archive}",
        "https://s3.amazonaws.com/drake-mirror/other/doxygen/{archive}",
    ],
    "github": [
        # For github.com, we choose a pattern based on the kind of commit.
        "https://github.com/{repository}/archive/refs/tags/{tag_name}.tar.gz",
        "https://github.com/{repository}/archive/{commit_sha}.tar.gz",
        # For Drake's mirrors, we use a single pattern no matter the commit.
        "https://drake-mirror.csail.mit.edu/github/{repository}/{commit}.tar.gz",  # noqa
        "https://s3.amazonaws.com/drake-mirror/github/{repository}/{commit}.tar.gz",  # noqa
    ],
    "github_release_attachments": [
        "https://github.com/{repository}/releases/download/{commit}/{filename}",
        "https://drake-mirror.csail.mit.edu/github/{repository}/{commit}/{filename}",  # noqa
        "https://s3.amazonaws.com/drake-mirror/github/{repository}/{commit}/{filename}",  # noqa
    ],
    "maven": [
        "https://jcenter.bintray.com/{fulljar}",
        "https://repo1.maven.org/maven2/{fulljar}",
        # N.B. ibiblio doesn't offer https.
        "http://maven.ibiblio.org/maven2/{fulljar}",
    ],
    "mosek": [
        "https://download.mosek.com/{path}",
        "https://drake-mirror.csail.mit.edu/mosek/{path}",
        "https://s3.amazonaws.com/drake-mirror/mosek/{path}",
    ],
}
