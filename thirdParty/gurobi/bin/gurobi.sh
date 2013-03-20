#!/bin/sh

if test -z "${GUROBI_HOME}" ; then
  echo
  echo "Environment variable GUROBI_HOME is not set.  Consult the Gurobi"
  echo "Quick Start Guide for information on how to set it."
  echo
fi

export PATH=$GUROBI_HOME/bin:$PATH
export LD_LIBRARY_PATH=$GUROBI_HOME/lib:$LD_LIBRARY_PATH
export PYTHONHOME=$GUROBI_HOME
export PYTHONPATH=$GUROBI_HOME:$PYTHONPATH

export PYTHONSTARTUP=$PYTHONHOME/lib/gurobi.py

$PYTHONHOME/bin/python2.7 $*
