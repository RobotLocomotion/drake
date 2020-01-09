"""Common helpers for source-tree linter utilities.
"""

import os
import sys


def find_all_sources(workspace_name):
    """Return [workspace, paths] list, where `workspace` is a path to the root
    of the given `workspace_name`, and `paths` are relative paths under it that
    are all of `workspace_name`'s source files, excluding third_party files.
    Because this abuses (escapes from) the Bazel sandbox, this function should
    *only* be used by linter tools and their unit tests.  It is thus given
    private visibility in our BUILD.bazel file.
    """
    # Our outermost `myprogram.runfiles` directory will contain a file named
    # MANIFEST.  Because this py_library declares a `data=[]` dependency on
    # the top-level .bazelproject file, the manifest will cite the original
    # location of that file, which we can abuse to find the absolute path to
    # the root of the source tree.  (For workspace_name values other than
    # "drake", callers should declare a data dependency on their workspace's
    # top-level .bazelproject file.)
    workspace_root = None
    for entry in sys.path:
        if workspace_root is not None:
            break
        if not entry.endswith(".runfiles"):
            continue
        manifest = os.path.join(entry, "MANIFEST")
        if not os.path.exists(manifest):
            continue
        with open(manifest, "r") as infile:
            lines = infile.readlines()
        for one_line in lines:
            if not one_line.startswith(workspace_name + "/.bazelproject"):
                continue
            _, source_sentinel = one_line.split(" ")
            workspace_root = os.path.dirname(os.path.realpath(source_sentinel))
            assert workspace_root.startswith("/"), workspace_root
            assert os.path.isdir(workspace_root), workspace_root
            break
    if not workspace_root:
        raise RuntimeError("Cannot find .bazelproject in MANIFEST")
    # Make sure we found the right place.
    workspace_file = os.path.join(workspace_root, "WORKSPACE")
    if not os.path.exists(workspace_file):
        raise RuntimeError(f"Cannot find WORKSPACE at {workspace_root}")
    required_line = f'workspace(name = "{workspace_name}")'
    with open(workspace_file, "r") as f:
        if (required_line + "\n") not in f.readlines():
            raise RuntimeError(
                f"Cannot find {required_line} in {workspace_file}")
    # Walk the tree (ignoring symlinks), and collect a list of all workspace-
    # relative filenames, but excluding a few specific items.
    relpaths = []
    for abs_dirpath, dirs, files in os.walk(workspace_root):
        assert abs_dirpath.startswith(workspace_root)
        rel_dirpath = abs_dirpath[len(workspace_root) + 1:]
        # Take all files within the currently-walked directory.
        for one_filename in files:
            if one_filename == ".DS_Store":
                continue
            relpaths.append(os.path.join(rel_dirpath, one_filename))
        # Don't recurse into children of "third_party".
        if abs_dirpath.endswith("/third_party"):
            dirs[:] = ()
            continue
        # Don't recurse into dotfile directories (such as ".git"), nor into
        # build directories.
        for i, one_dir in reversed(list(enumerate(list(dirs)))):
            if one_dir.startswith("."):
                dirs.pop(i)
            elif rel_dirpath == "" and one_dir.startswith("bazel-"):
                dirs.pop(i)
    return workspace_root, sorted(relpaths)
