def check_lists_consistency(
        *,
        files,
        glob_include,
        glob_exclude = None):
    """Checks that a hard-coded list of files fully covers a glob expression.

    When a package.BUILD.bazel file hard-codes a list of files (e.g., a list of
    headers of source files), we would like to fail-fast when upstream adds new
    files so that we can refresh our list. This is especially important when
    the version of the external is updated.

    Args:
        files (:obj:`list` of :obj:`str`): List of expected file names that
            will be matched by the glob expressions.
        glob_include (:obj:`list` of :obj:`str`): List of glob patterns to
            search for, per native.glob(include = ...).
        glob_exclude (:obj:`list` of :obj:`str`): List of glob patterns to
            exclude from the search, per native.glob(exclude = ...).
    """
    all_files = native.glob(glob_include, exclude = (glob_exclude or []))
    uncovered_files = sorted([x for x in all_files if x not in files])
    if len(uncovered_files) != 0:
        fail("The following files matched a glob of upstream sources, but " +
             "were not covered by the package.BUILD.bazel file: {}".format(
                 uncovered_files,
             ))
