#!/bin/bash

base_dir=$1

if [ ! -f $base_dir/software/tobuild.txt ]
then
    echo "First argument should be path to drc svn repository checkout."
    exit 1
fi

cd $base_dir

# remove all non-svn files
# command line from: http://stackoverflow.com/questions/4515586/clean-an-svn-checkout-remove-non-svn-files
svn status --no-ignore | grep '^[?I]' |  sed "s/^[?I] //" | xargs -I{} rm -rf "{}"
