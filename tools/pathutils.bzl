#==============================================================================
#BEGIN internal helpers

#------------------------------------------------------------------------------
# Remove prefix from path.
def ___remove_prefix(path, prefix):
    # If the prefix has more parts than the path, failure is certain.
    if len(prefix) > len(path):
        return None

    # Iterate over components to determine if a match exists.
    for n in range(len(prefix)):
        if prefix[n] == path[n]:
            continue
        elif prefix[n] == "*":
            continue
        else:
            return None

    return "/".join(path[len(prefix):])

def __remove_prefix(path, prefix):
    # Ignore trailing empty element (happens if prefix string ends with "/").
    if len(prefix[-1]) == 0:
        prefix = prefix[:-1]

    # If the prefix has more parts than the path, failure is certain. (We also
    # need at least one component of the path left over so the stripped path is
    # not empty.)
    if len(prefix) > (len(path) - 1):
        return None

    # Iterate over components to determine if a match exists.
    for n in range(len(prefix)):
        # Same path components match.
        if prefix[n] == path[n]:
            continue

        # Single-glob matches any (one) path component.
        elif prefix[n] == "*":
            continue

        # Mulit-glob matches one or more components.
        elif prefix[n] == "**":
            # If multi-glob is at the end of the prefix, return the last path
            # component.
            if n + 1 == len(prefix):
                return path[-1]

            # Otherwise, the most components the multi-glob can match is the
            # remaining components (len(prefix) - n - 1; the 1 is the current
            # prefix component) less one (since we need to keep at least one
            # component of the path).
            k = len(path) - (len(prefix) - n - 2)

            # Try to complete the match, iterating (backwards) over the number
            # of components that the multi-glob might match.
            for t in reversed(range(n, k)):
                x = ___remove_prefix(path[t:], prefix[n + 1:])
                if x != None:
                    return x

            # Multi-glob failed to match.
            return None

        else:
            # Components did not match.
            return None

    return "/".join(path[len(prefix):])

def _remove_prefix(path, prefix):
    """Remove prefix from path.

    This attempts to remove the specified prefix from the specified path. The
    prefix may contain the globs ``*`` or ``**``, which match one or many
    path components, respectively. Matching is greedy. Globs may only be
    matched against complete path components (e.g. ``a/*/`` is okay, but
    ``a*/`` is not treated as a glob and will be matched literally). Due to
    Skylark limitations, at most one ``**`` may be matched.

    Args:
        path (:obj:`str`) The path to modify.
        prefix (:obj:`str`) The prefix to remove.

    Returns:
        :obj:`str`: The path with the prefix removed if successful, or None if
        the prefix does not match the path.
    """
    return __remove_prefix(path.split("/"), prefix.split("/"))

#END internal helpers
#==============================================================================
#BEGIN macros

#------------------------------------------------------------------------------
def dirname(path):
    """Return the directory portion of a file path."""
    if path == "/":
        return "/"

    parts = path.split("/")

    if len(parts) > 1:
        return "/".join(parts[:-1])

    return "."

#------------------------------------------------------------------------------
def join_paths(*args):
    """Join paths without duplicating separators.

    This is roughly equivalent to Python's `os.path.join`.

    Args:
        \*args (:obj:`list` of :obj:`str`): Path components to be joined.

    Returns:
        :obj:`str`: The concatenation of the input path components.
    """
    result = ""

    for part in args:
        if part.endswith("/"):
            part = part[-1]

        if part == "" or part == ".":
            continue

        result += part + "/"

    return result[:-1]

#------------------------------------------------------------------------------
def output_path(ctx, input_file, strip_prefix, package_root = None):
    """Compute "output path".

    This computes the adjusted output path for an input file. Specifically, it
    a) determines the path relative to the invoking context (which is usually,
    but not always, the same as the path as specified by the user when the file
    was mentioned in a rule), without Bazel's various possible extras, and b)
    optionally removes prefixes from this path. When removing prefixes, the
    first matching prefix is removed.

    This is used primarily to compute the output install path, without the
    leading install prefix, for install actions.

    For example::

        install_files(
            dest = "docs",
            files = ["foo/bar.txt"],
            strip_prefix = ["foo/"],
            ...)

    The :obj:`File`'s path components will have various Bazel bits added. Our
    first step is to recover the input path, ``foo/bar.txt``. Then we remove
    the prefix ``foo``, giving a path of ``bar.txt``, which will become
    ``docs/bar.txt`` when the install destination is added.

    The input file must belong to the current package; otherwise, ``None`` is
    returned.

    Args:
        input_file (:obj:`File`): Artifact to be installed.
        strip_prefix (:obj:`list` of :obj:`str`): List of prefixes to strip
            from the input path before prepending the destination.

    Returns:
        :obj:`str`: The install destination path for the file.
    """

    if package_root == None:
        # Determine base path of invoking context.
        package_root = join_paths(ctx.label.workspace_root, ctx.label.package)

    # Determine effective path by removing path of invoking context and any
    # Bazel output-files path.
    input_path = input_file.path
    if input_file.is_source:
        input_path = _remove_prefix(input_path, package_root)
    else:
        out_root = join_paths("bazel-out/*/*", package_root)
        input_path = _remove_prefix(input_path, out_root)

    # Deal with possible case of file outside the package root.
    if input_path == None:
        return None

    # Possibly remove prefixes.
    for p in strip_prefix:
        output_path = _remove_prefix(input_path, p)
        if output_path != None:
            return output_path

    return input_path

#END macros
