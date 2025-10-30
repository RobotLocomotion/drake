"""Common helpers for source-tree linter utilities."""

import os
from pathlib import Path

from python import runfiles


def find_all_sources():
    """Return [workspace, paths] list, where `workspace` is a path to the root
    of Drake's source tree and `paths` are relative paths under it that are all
    of `workspace_name`'s source files, excluding third_party files.
    Because this abuses (escapes from) the Bazel sandbox, this function should
    *only* be used by linter tools and their unit tests.  It is thus given
    private visibility in our BUILD.bazel file.
    """
    manifest = runfiles.Create()
    dotfile = Path(manifest.Rlocation("drake/.bazelproject"))
    workspace_root = dotfile.absolute().resolve().parent

    # Walk the tree (ignoring symlinks), and collect a list of all workspace-
    # relative filenames, but excluding a few specific items.
    relpaths = []
    for abs_dirpath, dirs, files in os.walk(str(workspace_root)):
        # TODO(jwnimmer-tri) Once we drop Jammy use Path.walk (vs os.walk),
        # and remove this extra conversion.
        abs_dirpath = Path(abs_dirpath)
        # Take all files within the currently-walked directory.
        rel_dirpath = abs_dirpath.relative_to(workspace_root)
        for one_filename in files:
            if one_filename == ".DS_Store":
                continue
            if rel_dirpath == "." and one_filename.startswith("bazel-"):
                continue
            relpaths.append(str(rel_dirpath / one_filename))
        # Don't recurse into subdirectories of "third_party" (but do still
        # return files directly in "third_party").
        if abs_dirpath.name == "third_party":
            dirs[:] = ()
            continue
        # Don't recurse into dotfile directories (such as ".git").
        for i, one_dir in reversed(list(enumerate(list(dirs)))):
            if one_dir.startswith("."):
                dirs.pop(i)
    return workspace_root, sorted(relpaths)
