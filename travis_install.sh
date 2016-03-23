set -e
brew tap homebrew/python > tap.log || cat tap.log
brew tap homebrew/science > tap.log || cat tap.log
brew update > update.log || cat update.log
brew install autoconf automake ccache cmake doxygen gcc glib graphviz gtk+ \
  jpeg libpng libtool mpfr mpich2 numpy python qt swig valgrind vtk5 \
  wget > install.log  || (cat install.log && exit 1)
mkdir -p $TRAVIS_BUILD_DIR/build
export CMAKE_FLAGS="-DWITH_AVL:BOOL=ON -DWITH_DIRECTOR:BOOL=OFF -DWITH_IRIS:BOOL=ON -DWITH_LIBBOT:BOOL=ON -DWITH_MOSEK:BOOL=ON -DWITH_OCTOMAP:BOOL=ON -DWITH_SPOTLESS:BOOL=OFF -DWITH_XFOIL:BOOL=ON"
make download-all > download.log || (cat download.log && exit 1)
make
