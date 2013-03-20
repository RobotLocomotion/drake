#!/usr/bin/python -i

import sys

from gurobipy import *

version = str(gurobi.version()[0]) + '.' + \
          str(gurobi.version()[1]) + '.' + \
          str(gurobi.version()[2])
platform = gurobi.platform()

print('\nGurobi Interactive Shell (' + platform + '), Version ' + version)
print('Copyright (c) 2013, Gurobi Optimization, Inc.')
print('Type "help()" for help\n')

sys.ps1 = "gurobi> "
sys.ps2 = "....... "
