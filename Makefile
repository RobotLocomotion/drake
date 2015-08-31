
BUILD_SYSTEM:=$(OS)
ifeq ($(BUILD_SYSTEM),Windows_NT)
BUILD_SYSTEM:=$(shell uname -o 2> uname.err || echo Windows_NT) # set to Cygwin if appropriate
else
BUILD_SYSTEM:=$(shell uname -s)
endif
BUILD_SYSTEM:=$(strip $(BUILD_SYSTEM))

# Figure out where to build the software.
#   Use BUILD_PREFIX if it was passed in.
#   If not, search up to four parent directories for a 'build' directory.
#   Otherwise, use ./build.
ifeq ($(BUILD_SYSTEM), Windows_NT)
ifeq "$(BUILD_PREFIX)" ""
BUILD_PREFIX:=$(shell (for %%x in (. .. ..\.. ..\..\.. ..\..\..\..) do ( if exist %cd%\%%x\build ( echo %cd%\%%x\build & exit ) )) & echo %cd%\build )
endif
# don't clean up and create build dir as I do in linux.  instead create it during configure.
else
ifeq "$(BUILD_PREFIX)" ""
BUILD_PREFIX:=$(shell for pfx in ./ .. ../.. ../../.. ../../../..; do d=`pwd`/$$pfx/build;\
               if [ -d $$d ]; then echo $$d; exit 0; fi; done; echo `pwd`/build)
endif
# create the build directory if needed, and normalize its path name
BUILD_PREFIX:=$(shell mkdir -p $(BUILD_PREFIX) && cd $(BUILD_PREFIX) && echo `pwd`)
endif

ifeq "$(BUILD_SYSTEM)" "Cygwin"
  BUILD_PREFIX:=$(shell cygpath -m $(BUILD_PREFIX))
endif

# Default to a release build.  If you want to enable debugging flags, run
# "make BUILD_TYPE=Debug"
ifeq "$(BUILD_TYPE)" ""
  BUILD_TYPE="Release"
endif

override CMAKE_FLAGS:=$(shell echo $(CMAKE_FLAGS))
# The line above is to help passing through multiple CMAKE_FLAGS from the parent.
# Previously, sending in more than one command was not supported because it required
# quotes in the CMakeLists.txt and then the it was acting as only a single argument
# to cmake.  This is an age-old problem (see c.f. http://stackoverflow.com/a/9484942 ).
# For the more complicated case, where you want to pass in a string that needs quotes,
# can still handle it in your CMakeLists.txt by using, e.g.
# string(REPLACE \" \\\" CMAKE_FLAGS_FROM_ENV "$ENV{CMAKE_FLAGS}")  # turn " into \" for passing through
# and then
# BUILD_COMMAND ${PODS_MAKE_COMMAND} CMAKE_FLAGS="${CMAKE_FLAGS_FROM_ENV} -DWITH_SNOPT=ON -DWITH_BULLET=OFF" BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE}

.PHONY: all
all: pod-build/Makefile
	cmake --build pod-build --config $(BUILD_TYPE)

pod-build/Makefile:
	"$(MAKE)" configure

.PHONY: options
options: configure
ifeq ($(OS),Windows_NT)
	cmake-gui pod-build
else
	ccmake pod-build
endif

.PHONY: configure
configure:
#	@echo "BUILD_SYSTEM: '$(BUILD_SYSTEM)'"
	@echo "BUILD_PREFIX: $(BUILD_PREFIX)"

# create the temporary build directory if needed
# create the lib directory if needed, so the pkgconfig gets installed to the right place
ifeq ($(BUILD_SYSTEM), Windows_NT)
	@if not exist pod-build ( mkdir pod-build )
else
	@mkdir -p pod-build
endif

# run CMake to generate and configure the build scripts
	@echo "Configuring with CMAKE_FLAGS: $(CMAKE_FLAGS)"
	@cd pod-build && cmake $(CMAKE_FLAGS) -DCMAKE_INSTALL_PREFIX=$(BUILD_PREFIX) \
	       	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) ..

.PHONY: download-all
download-all: configure
	cmake --build pod-build --config $(BUILD_TYPE) --target $@

.PHONY: clean
clean:
ifeq ($(BUILD_SYSTEM),Windows_NT)
	rd /s pod-build
	rd /s build
else
	-if [ -e pod-build/install_manifest.txt ]; then rm -f `cat pod-build/install_manifest.txt`; fi
	-if [ -d pod-build ]; then cmake --build pod-build --target clean-all; fi #rm -rf pod-build; fi
	-rm -rf build
endif

# other (custom) targets are passed through to the cmake-generated Makefile
%::
	cmake --build pod-build --config $(BUILD_TYPE) --target $@

# Default to a less-verbose build.  If you want all the gory compiler output,
# run "make VERBOSE=1"
$(VERBOSE).SILENT:
