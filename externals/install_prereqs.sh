#!/bin/bash


case $1 in
  ("homebrew")
    brew install cmake pkg-config gtk+ ;;
  ("macports")
    port install cmake gtk2 ;;
  ("ubuntu")
    apt-get install cmake openjdk-6-jdk build-essential ;;
  ("cygwin")
    cygwin-setup -q -P make pkg-config ;;
  (*)
    echo "Usage: ./install_prereqs.sh package_manager"
    echo "where package_manager is one of the following: "
    echo "  homebrew"
    echo "  macports"
    echo "  ubuntu"
    echo "  cygwin"
    exit 1 ;;
esac

if [ -e "../pod-build/drake_external_source_dirs.txt" ]
then
  SUBDIRS=`cat ../pod-build/drake_external_source_dirs.txt`
else
  echo "install_prereqs could not find the list of external source directories. Please run 'make download-all' first." 1>&2
  exit 1
fi

for subdir in $SUBDIRS; do
  if [ -f $subdir/install_prereqs.sh ]; then
    echo "installing prereqs for $subdir"
    ( cd $subdir; ./install_prereqs.sh $1 || true )
  fi
done
