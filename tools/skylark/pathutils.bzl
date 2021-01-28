def _match_single_glob_tokens(path_tokens, prefix_tokens):
    """If the prefix matches the path (anchored at the start), returns the
    segment of the path tokens that matched -- or None if no match.  The
    arguments are lists of strings, with an implied "/" between elements.

    The token "*" must match exactly one path token.
    The token "**" is not allowed.
    """
    if len(prefix_tokens) > len(path_tokens):
        # The prefix is too long ==> no match.
        return None

    # Check the tokens pairwise (stopping at the shorter of the two lists).
    for prefix, path in zip(prefix_tokens, path_tokens):
        if prefix == "*":
            # The "*" matches anything.
            continue
        if prefix != path:
            # Mismatch.
            return None

    # Successful match.
    return path_tokens[:len(prefix_tokens)]

def _match_double_glob_tokens(path_tokens, prefix_tokens):
    """If the prefix matches the path (anchored at the start), returns the
    segment of the path tokens that matched -- or None if no match.  The
    arguments are lists of strings, with an implied "/" between elements.

    The token "*" must match exactly one path token.
    The token "**" match any number of path tokens, greedily.
    """

    # Expand the double ("**") globs into a list of brute-force candidates,
    # i.e., ["**"] ==> [], ["*"], ["*", "*"],  ["*", "*", "*"], etc. up to the
    # most that we could need.  To produce greedy matching, we rank them from
    # longest to shorest, and expand the earlier globs first.  Each candidate
    # is a prefix token list, with either a literal str or a "*" in each item.
    candidates = [[]]
    for prefix_token in prefix_tokens:
        # For a literal token or a "*", append it to every candidate.
        if prefix_token != "**":
            for i in range(len(candidates)):
                candidates[i].append(prefix_token)
            continue

        # For a ** token, replicate the candidates for possible ** matches.
        # The longest ** match should be as long as the whole path.
        expansions = [
            ["*"] * i
            for i in reversed(range(len(path_tokens) + 1))
        ]
        new_candidates = [
            candidate + expansion
            for candidate in candidates
            for expansion in expansions
        ]
        candidates = new_candidates

    # Check each candidate prefix token list for a match against the path.  The
    # first candidate that matches, wins.
    for candidate_tokens in candidates:
        match = _match_single_glob_tokens(path_tokens, candidate_tokens)
        if match != None:
            return match

    # Nothing matched.
    return None

def _remove_prefix(path, prefix):
    """Remove prefix from path.

    This attempts to remove the specified prefix from the specified path. The
    prefix may contain the globs ``*`` or ``**``, which match one or many
    path components, respectively. Matching is greedy. Globs may only be
    matched against complete path components (e.g. ``a/*/`` is okay, but
    ``a*/`` is not treated as a glob and will be matched literally).

    Args:
        path (:obj:`str`) The path to modify.
        prefix (:obj:`str`) The prefix to remove.

    Returns:
        :obj:`str`: The path with the prefix removed if successful, or None if
        the prefix does not match the path.
    """
    path_tokens = path.split("/")
    prefix_tokens = prefix.split("/")

    # Ignore trailing empty element (happens if prefix string ends with "/").
    if len(prefix_tokens[-1]) == 0:
        prefix_tokens = prefix_tokens[:-1]

    # Match the prefix against the path, leaving the final path name intact.
    match = _match_double_glob_tokens(path_tokens[:-1], prefix_tokens)

    # If a match was found, return the stripped path, else None
    if match == None:
        return None
    return "/".join(path_tokens[len(match):])

def basename(path):
    """Return the file name portion of a file path."""
    return path.split("/")[-1]

def dirname(path):
    """Return the directory portion of a file path."""
    if path == "/":
        return "/"

    parts = path.split("/")

    if len(parts) > 1:
        return "/".join(parts[:-1])

    return "."

def join_paths(*args):
    """Join paths without duplicating separators.

    This is roughly equivalent to Python's `os.path.join`.

    Args:
        *args (:obj:`list` of :obj:`str`): Path components to be joined.

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
