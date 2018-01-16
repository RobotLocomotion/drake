# -*- python -*-

def _dirname(p, remove = 1):
    # Returns parent directory name for a path `p`.
    pieces = p.split('/')
    return '/'.join(pieces[0:-remove])

def get_drake_path(workspace_dir):
    """Returns path of Drake. This is replaced under
    `external_data_workspace_test.sh`.
    """
    return _dirname(workspace_dir, 4)
