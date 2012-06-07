#!/bin/bash

## deploy.sh - a bash script that generates a 
## deployable PUBLIC bundle of Drake.

## DO NOT RUN THIS FROM THE TRUNK!!!!!
## You will need to run this from a working copy
## of the "drake" branch of robotlib.

## You need to always make sure that the following
## number actually matches the last version
## before copy - svn log --verbose --stop-on-copy
## will tell you what that is

SVN_LAST_REV_BEFORE_COPY=2845
SVN_SRC="https://svn.csail.mit.edu/locomotion/robotlib/trunk"

## TODO: change this once script is working
## this needs to be a working copy of the 
## "drake" branch.

SVN_WORKING_COPY=`pwd`

## TODO: use credentials here. probably won't need this
## if we run from Hudson, or manually.
# SVN_USERNAME=""
# SNV_PASSWORD=""

## start by getting trunk merged over to us
echo "Going to svn merge trunk over to us..."
cd $SVN_WORKING_COPY
svn merge -r $SVN_LAST_REV_BEFORE_COPY:HEAD $SVN_SRC 
echo "Done."

## Remove .svn directories.
echo "Removing all SVN housekeeping files..."
find . -depth -type d -name .svn -exec rm -rf {} \;
echo "Done."

# Remove dev directories
echo "Removing all dev directories..."
find . -depth -type d -name dev -exec rm -rf {} \;
echo "Done."

## Remove NORELEASE directories
echo "Getting rid of NORELEASE directories..."
for dir in `find . -type d -print`;
do
    if [ -e $dir/.NORELEASE ]
	then
	echo "Removing $dir"
	rm -rf $dir
    fi
done
echo "Done."

## Remove NORELEASE files
echo "Getting rid of NORELEASE files..."
for file in `find . -type f` 
do head -n5 $file | grep -q NORELEASE && rm $file
done

echo "Done."
echo "Release stripping is complete."

echo "Making a .tar.gz now..."
cd ../
tar cvf /tmp/drake.tar drake && gzip /tmp/drake.tar
echo "Done."
echo "The tar.gz is now at /tmp/drake.tar.gz."

echo "Making a .zip file now..."
zip -r /tmp/drake.zip drake
echo "Done."
echo "The zipfile is now at /tmp/drake.zip."

echo "All packaging is complete. Enjoy!"