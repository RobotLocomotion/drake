#!/usr/bin/env python3

import json
import sys
import urllib.request


def get_latest_version(name):
    r = urllib.request.urlopen(f'https://pypi.org/pypi/{name}/json')
    d = json.loads(r.read())
    return d['info']['version']


def update_requirements(requirements_path):
    updated = False

    lines = []
    with open(requirements_path, 'r') as f:
        for line in f:
            line = line.strip()

            if not line.startswith('#') and '==' in line:
                name, current = line.split('==')
                latest = get_latest_version(name)
                if latest != current:
                    line = f'{name}=={latest}'
                    updated = True

            lines.append(line)

    if updated:
        with open(requirements_path, 'w') as f:
            f.write('\n'.join(lines))
        print(requirements_path)


def main(args):
    for p in args:
        update_requirements(p)


if __name__ == '__main__':
    main(sys.argv[1:])
