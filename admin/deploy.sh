#!/bin/bash

## deploy.sh - a bash script that generates a 
## deployable PUBLIC bundle of Drake.

## start by getting trunk merged over to us
echo "I am about to create a directory here called workspace and check Drake out into it."
echo "Checking out HEAD from Drake..."
mkdir workspace
cd workspace
svn co https://svn.csail.mit.edu/locomotion/robotlib/trunk
mv trunk drake
cd drake
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

## Remove SNOPT7 we provide
echo "Removing the SNOPT7 we provide for internal users..."
rm -rf thirdParty/snopt7
echo "Done."


echo "All done."
echo "Release stripping is complete."

echo "Making a .tar.gz now..."
cd ../
tar cvf drake.tar drake && gzip drake.tar
echo "Done."
echo "The tar.gz is now at drake.tar.gz."

echo "Making a .zip file now..."
zip -r drake.zip drake
echo "Done."
echo "The zipfile is now at drake.zip."

echo "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-="
echo "           All packaging is complete. Enjoy!"
echo "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-="