# Default pod makefile distributed with pods version: 12.11.14

# Figure out where to build the software.
#   Use BUILD_PREFIX if it was passed in.
#   If not, search up to four parent directories for a 'build' directory.
#   Otherwise, use ./build.
ifeq "$(BUILD_PREFIX)" ""
BUILD_PREFIX:=$(shell for pfx in ./ .. ../.. ../../.. ../../../..; do d=`pwd`/$$pfx/build;\
               if [ -d $$d ]; then echo $$d; exit 0; fi; done; echo `pwd`/build)
endif
# create the build directory if needed, and normalize its path name
BUILD_PREFIX:=$(shell mkdir -p $(BUILD_PREFIX) && cd $(BUILD_PREFIX) && echo `pwd`)

# Default to a release build.  If you want to enable debugging flags, run
# "make BUILD_TYPE=Debug"
ifeq "$(BUILD_TYPE)" ""
BUILD_TYPE="Release"
endif

# force cmake configure if the BUILD_PREFIX doesn't match the cmake cache
CMAKE_INSTALL_PREFIX=$(shell cd pod-build 2> /dev/null && cmake -L 2> /dev/null | grep CMAKE_INSTALL_PREFIX | cut -d "=" -f2 | tr -d '[:space:]')
ifneq "$(BUILD_PREFIX)" "$(CMAKE_INSTALL_PREFIX)"
	OUT:=$(shell echo "\nBUILD_PREFIX $(BUILD_PREFIX) does not match CMAKE cache $(CMAKE_INSTALL_PREFIX).  Forcing configure\n\n"; touch CMakeLists.txt)
endif

all: pod-build/Makefile
	$(MAKE) -C pod-build all install

pod-build/Makefile:
	$(MAKE) configure

.PHONY: configure
configure:
	@echo "\nBUILD_PREFIX: $(BUILD_PREFIX)\n\n"

	# create the temporary build directory if needed
	@mkdir -p pod-build
	@cd pod-build && ln -sf ../CTestCustom.cmake  # actually has to live in the build path

	# run CMake to generate and configure the build scripts
	@cd pod-build && cmake -DCMAKE_INSTALL_PREFIX=$(BUILD_PREFIX) \
		   -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) ..

.PHONY: doc doxygen 
doc:	doxygen doc/drake.pdf doc/urdf/drakeURDF.html

doxygen :
	doxygen doc/Doxyfile

doc/drake.pdf :
	# apologies for hard-coding this for my mac for now... - Russ
	cd doc && make -f ~/code/latex/makefile_tex drake.pdf

doc/urdf/drakeURDF.html : doc/drakeURDF.xsd
	# apologies for hard-coding this for my mac for now... - Russ
	cd doc && /Applications/oxygen/schemaDocumentationMac.sh drakeURDF.xsd -cfg:oxygen_export_settings_html.xml

.PHONY: mlint
mlint	:
	matlab -nodisplay -r "addpath(fullfile(pwd,'thirdParty','runmlint')); runmlint('.mlintopts'); exit"

test	:  configure
	-@cd pod-build && ctest -D Experimental --output-on-failure --timeout 300

test_continuous : configure
	while true; do $(MAKE) Continuous; sleep 300; done

.PHONY: check_prereqs install_prereqs_macports install_prereqs_homebrew install_prereqs_ubuntu
check_prereqs: 
	if javac -source 1.6 -version > /dev/null 2> /dev/null; then exit 0; else echo "ERROR: Java 6 (or greater) sdk is required."; exit 1; fi  # test for javac >= 1.6

install_prereqs_macports : check_prereqs
	port install graphviz

install_prereqs_homebrew : check_prereqs
	brew install boost graphviz

install_prereqs_ubuntu : check_prereqs
	apt-get install graphviz

clean:
	-if [ -e pod-build/install_manifest.txt ]; then rm -f `cat pod-build/install_manifest.txt`; fi
	-if [ -d pod-build ]; then $(MAKE) -C pod-build clean; rm -rf pod-build; fi

# other (custom) targets are passed through to the cmake-generated Makefile 
%::
	$(MAKE) -C pod-build $@

# Default to a less-verbose build.  If you want all the gory compiler output,
# run "make VERBOSE=1"
$(VERBOSE).SILENT:

