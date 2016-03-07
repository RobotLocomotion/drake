set -e
export CC=$DRAKE_CC
export CXX=$DRAKE_CXX
$CC -v
mkdir -p $TRAVIS_BUILD_DIR/build
if [ "$TRAVIS_OS_NAME" = "linux" ]; then
    export DEBIAN_FRONTEND=noninteractive
    sudo -E apt-get update -qq
    export CMAKE_FLAGS="-DWITH_AVL:BOOL=ON -DWITH_DIRECTOR:BOOL=ON -DWITH_IRIS:BOOL=ON -DWITH_LIBBOT:BOOL=ON -DWITH_MOSEK:BOOL=ON -DWITH_OCTOMAP:BOOL=ON -DWITH_SPOTLESS:BOOL=OFF -DWITH_SWIG_MATLAB=ON -DWITH_XFOIL:BOOL=ON"
    make download-all
    sudo -E ./install_prereqs.sh ubuntu
elif [ "$TRAVIS_OS_NAME" = "osx" ]; then
    brew update > brew_update.log
    export CMAKE_FLAGS="-DWITH_AVL:BOOL=ON -DWITH_DIRECTOR:BOOL=OFF -DWITH_IRIS:BOOL=ON -DWITH_LIBBOT:BOOL=ON -DWITH_MOSEK:BOOL=ON -DWITH_OCTOMAP:BOOL=ON -DWITH_SPOTLESS:BOOL=OFF -DWITH_XFOIL:BOOL=ON"
    make download-all > download.log || (cat download.log && exit 1)
    ./install_prereqs.sh homebrew
else
    exit 1
fi
make
