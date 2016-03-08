set -e
export PYTHONPATH=$TRAVIS_BUILD_DIR/build/lib/python2.7/dist-packages:$PYTHONPATH
cd $TRAVIS_BUILD_DIR/drake/pod-build
ctest --build-config Release --dashboard Experimental --output-on-failure --parallel 1 --timeout 1000
