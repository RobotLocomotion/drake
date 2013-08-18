#!/bin/bash

environmentFile=$1
ctestFile=$2

source $environmentFile
ctest -S $ctestFile -VV

