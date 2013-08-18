#!/usr/bin/env python

import os
import sys
from xml.dom import minidom

def replaceTesting(fileA, fileB, outFile):

    assert os.path.isfile(fileA)
    assert os.path.isfile(fileB)

    docA = minidom.parse(fileA)
    docB = minidom.parse(fileB)

    testingTagName = 'Testing'

    testingElementA = docA.getElementsByTagName(testingTagName)
    testingElementB = docB.getElementsByTagName(testingTagName)

    assert len(testingElementA) == 1
    assert len(testingElementB) == 1

    testingElementA = testingElementA[0]
    testingElementB = testingElementB[0]

    testingElementA.childNodes = testingElementB.childNodes

    print 'writing: ', outFile
    docA.writexml(open(outFile, 'wb'), encoding="UTF-8")


def main():

    if len(sys.argv) != 4:
        print 'Usage: %s <original xml file> <new xml file with testing info> <output file>' % sys.argv[0]
        sys.exit(1)


    replaceTesting(sys.argv[1], sys.argv[2], sys.argv[3])



if __name__ == '__main__':
    main()
