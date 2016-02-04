set -e

if [ "$CXX" = "g++" ]
	then 
	export CXX="g++-4.8" CC="gcc-4.8"
fi

install_vtk_homebrew_bottle()
{
  wget https://dl.dropboxusercontent.com/s/mtacwrjgmtanfaz/vtk5-5.10.1_2.mavericks.bottle.1.tar.gz
  brew tap homebrew/science
  brew install qt
  brew install vtk5-5.10.1_2.mavericks.bottle.1.tar.gz
}


if [ "$TRAVIS_OS_NAME" = "linux" ]
	then
	sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
	sudo apt-get update -qq
	sudo apt-get install gcc-4.8 g++-4.8
    export CMAKE_FLAGS="-DWITH_SPOTLESS:BOOL=OFF -DWITH_LIBBOT:BOOL=ON -DWITH_DIRECTOR:BOOL=ON -DWITH_IRIS:BOOL=ON -DWITH_OCTOMAP:BOOL=ON -DWITH_MOSEK:BOOL=ON -DWITH_AVL:BOOL=ON -DWITH_XFOIL:BOOL=ONi -DWITH_SWIG_MATLAB=ON"
	mkdir build
	make download-all
	sudo ./install_prereqs.sh ubuntu
elif [ "$TRAVIS_OS_NAME" = "osx" ]
	then 
	brew update > brew_update.log
	export CMAKE_FLAGS="-DWITH_SPOTLESS:BOOL=OFF -DWITH_LIBBOT:BOOL=ON -DWITH_DIRECTOR:BOOL=ON -DWITH_IRIS:BOOL=ON -DWITH_OCTOMAP:BOOL=ON -DWITH_MOSEK:BOOL=ON -DWITH_AVL:BOOL=ON -DWITH_XFOIL:BOOL=ON"
	mkdir build
	make download-all > download.log || (cat download.log && exit 1)
	install_vtk_homebrew_bottle
	./install_prereqs.sh homebrew
else
	echo "WARNING: TRAVIS_OS_NAME: $TRAVIS_OS_NAME is not handled, not installing prereqs"
	mkdir build
	make download-all
fi
