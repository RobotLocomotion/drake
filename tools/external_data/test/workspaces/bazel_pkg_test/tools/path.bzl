# -*- python -*-

def dirname(p, remove = 1):
    """Returns parent directory name for a path `p`.
    @param remove Number of child levels to remove. Default is 1.
    """
    pieces = p.split('/')
    return '/'.join(pieces[0:-remove])
