#!/bin/bash

## deploy.sh - a bash script that generates a 
## deployable PUBLIC bundle of Drake.

## configure this script by setting the following variables:
repository_location=https://svn.csail.mit.edu/locomotion/collections/drake-minimal
release_name=drake-minimal

## start by pulling a fresh copy from svn over to us
echo "I am about to create a directory here called workspace and check Drake out into it."
echo "Checking out HEAD from Drake..."
mkdir workspace
cd workspace
svn co $repository_location $release_name
cd $release_name
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
do grep -q NORELEASE $file && rm $file
done
echo "Done."

echo "All done."
echo "Release stripping is complete."

echo "Making a .tar.gz now..."
cd ../
tar cvf $release_name.tar $release_name && gzip $release_name.tar
echo "Done."
echo "The tar.gz is now at $release_name.tar.gz."

echo "Making a .zip file now..."
zip -r $release_name.zip $release_name
echo "Done."
echo "The zipfile is now at $release_name.zip."

echo "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-="
echo "           All packaging is complete. Enjoy!"
echo "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-="
