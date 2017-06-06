import cps
import re

#------------------------------------------------------------------------------
def read_defs(pattern, path, groups = (1, 2)):
    defs = {}
    pattern = re.compile(pattern)

    with open(path) as f:
        for line in f:
            match = pattern.match(line)
            if match is not None:
                defs[match.group(groups[0])] = match.group(groups[1])

    return defs

#------------------------------------------------------------------------------
def read_version_defs(pattern, path):
    pattern = re.compile(pattern)

    with open(path) as f:
        for line in f:
            match = pattern.match(line)
            if match is not None:
                defs = {}
                defs['VERSION_MAJOR'] = match.group(1)
                defs['VERSION_MINOR'] = match.group(2)
                defs['VERSION_PATCH'] = match.group(3)
                return defs

    return {}

#------------------------------------------------------------------------------
def read_requires(paths):
    defs = {}

    for p in paths:
        package = cps.read(p)
        if package.version is not None:
            defs["%s_VERSION" % package.name] = package.version

    return defs
