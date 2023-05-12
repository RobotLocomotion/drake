#!/usr/bin/python3 -B
"""
Fix broken (non-physical) inertias in SDFormat/URDF,
by writing a revised file.
"""
import sys

# With `etree` we can't avoid losing comments outside the root element. If we
# care about those, we will likely have to revive a dependency on `lxml`.
from xml.etree.ElementTree import XMLParser, TreeBuilder, parse


def fix_inertias():
    infile = sys.argv[1]
    outfile = sys.argv[2]

    ctb = TreeBuilder(insert_comments=True, insert_pis=True)
    xp = XMLParser(target=ctb)
    tree = parse(infile, parser=xp)
    root = tree.getroot()
    tree.write(outfile, encoding='UTF-8', xml_declaration=True)

def main():
    fix_inertias()


if __name__ == '__main__':
    main()
