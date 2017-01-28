from __future__ import print_function

import numpy as np

from _autodiffutils import VectorXAutoDiffXd


def toAutoDiff(values, derivatives):
    return VectorXAutoDiffXd(np.asarray(values), np.asarray(derivatives))

if __name__ == '__main__':
    print(toAutoDiff(np.ones((3)), np.eye(3)))
