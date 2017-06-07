import cps
import re
import sys

# n.b. sys.argv[0] is the program name
version_file = sys.argv[1]
dependency_cps_files = sys.argv[2:]


def read_defs(pattern, groups=(1, 2)):
    defs = {}
    pattern = re.compile(pattern)

    with open(version_file) as f:
        for line in f:
            match = pattern.match(line)
            if match is not None:
                defs[match.group(groups[0])] = match.group(groups[1])

    return defs


def read_version_defs(pattern):
    pattern = re.compile(pattern)

    with open(version_file) as f:
        for line in f:
            match = pattern.match(line)
            if match is not None:
                defs = {}
                defs['VERSION_MAJOR'] = match.group(1)
                defs['VERSION_MINOR'] = match.group(2)
                defs['VERSION_PATCH'] = match.group(3)
                return defs

    return {}


def read_requires():
    defs = {}

    for p in dependency_cps_files:
        package = cps.read(p)
        if package.version is not None:
            defs["%s_VERSION" % package.name] = package.version

    return defs
