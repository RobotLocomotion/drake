#!/bin/bash
set -eux -o pipefail

# See: 
user=$(logname)
logname
sudo -u ${user} touch tmp_file
