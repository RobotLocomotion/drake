# -*- python -*-

def check_lists_consistency(
        glob,
        file_list):
    """Checks consistency between lists of files and glob expression.

    If lists of files are hard-coded (e.g. public and private headers), one may
    want to verify that all the files that should be listed are indeed listed.
    This is especially important when the version of the target is updated, to
    verify that the lists of files are also updated if necessary.

    Args:
        glob (:obj:`str`): expression used to find all the files. The list of
            files created thanks to this expression is compared to the given
            lists of files.
        file_list (:obj:`list` of :obj:`str`): List of file names. This is
        typically a list of public headers concatenated with a list of private
        headers.
    """
    all_headers = native.glob(glob)
    unknown_headers = [x for x in all_headers if x not in file_list]
    if len(unknown_headers) != 0:
        fail("Inconsistent file lists. Unknown file(s): " +
             str(unknown_headers))
