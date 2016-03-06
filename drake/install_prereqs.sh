#!/bin/bash

if javac -source 1.6 -version > /dev/null 2> /dev/null;
  then
    echo "java version >= 1.6: YES"
  else
    echo "ERROR: Java 6 (or greater) sdk is required."; exit 1;
fi  # test for javac >= 1.6

case $1 in
  ("homebrew")
    brew install graphviz ;;
  ("macports")
    port install graphviz ;;
  ("ubuntu")
    apt-get install graphviz cmake-curses-gui ;;
  ("cygwin")
    ;;
  (*)
    echo "Usage: ./install_prereqs.sh package_manager"
    echo "where package_manager is one of the following: "
    echo "  homebrew"
    echo "  macports"
    echo "  ubuntu"
    echo "  cygwin"
    exit 1 ;;
esac

if [ -f tobuild.txt ]; then
  SUBDIRS=`grep -v "^\#" tobuild.txt`
  for subdir in $SUBDIRS; do
    if [ -f $subdir/install_prereqs.sh ]; then
      echo "installing prereqs for $subdir"
      ( cd $subdir; ./install_prereqs.sh $1 || true )
    fi
  done
fi
