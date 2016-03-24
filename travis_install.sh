set -e
brew tap homebrew/python > tap.log
brew update > update.log
brew install glib graphviz gtk+ numpy python swig > install.log
mkdir -p $TRAVIS_BUILD_DIR/build
export CMAKE_FLAGS="-DWITH_AVL:BOOL=ON -DWITH_DIRECTOR:BOOL=OFF -DWITH_IRIS:BOOL=ON -DWITH_LIBBOT:BOOL=ON -DWITH_MOSEK:BOOL=ON -DWITH_OCTOMAP:BOOL=ON -DWITH_SPOTLESS:BOOL=OFF -DWITH_XFOIL:BOOL=ON"
make download-all > download.log || (cat download.log && exit 1)
make
