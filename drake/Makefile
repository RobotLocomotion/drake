
# support only a few flags that might be passed through:
#   BUILD_PREFIX, BUILD_TYPE, CMAKE_FLAGS*
CMAKE_FLAGS+=$(strip $(CMAKE_FLAGS1) $(CMAKE_FLAGS2) $(CMAKE_FLAGS3) $(CMAKE_FLAGS4) $(CMAKE_FLAGS5) $(CMAKE_FLAGS6) $(CMAKE_FLAGS7) $(CMAKE_FLAGS8) $(CMAKE_FLAGS9) $(CMAKE_FLAGS10) $(CMAKE_FLAGS11) $(CMAKE_FLAGS12) $(CMAKE_FLAGS13) $(CMAKE_FLAGS14) $(CMAKE_FLAGS15) $(CMAKE_FLAGS16) $(CMAKE_FLAGS17) $(CMAKE_FLAGS18) $(CMAKE_FLAGS19) $(CMAKE_FLAGS20))
ifneq "$(BUILD_PREFIX)" ""
  CMAKE_FLAGS+=-DCMAKE_INSTALL_PREFIX="$(BUILD_PREFIX)"
endif
ifeq "$(BUILD_TYPE)" ""
  BUILD_TYPE="Release"
endif
CMAKE_FLAGS+=-DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
CMAKE_CONFIG=--config $(BUILD_TYPE)

.PHONY: all
all: configure
	cmake --build pod-build $(CMAKE_CONFIG) --target install

pod-build:
	cmake -E make_directory pod-build

.PHONY: configure
configure: pod-build
	@echo Configuring with CMAKE_FLAGS: $(CMAKE_FLAGS)
	@cd pod-build && cmake $(CMAKE_FLAGS) ..

.PHONY: options
options: configure
ifeq ($(OS),Windows_NT)
	cmake-gui pod-build
else
	ccmake pod-build
endif

.PHONY: clean
clean:
	cmake --build pod-build --target clean
	cmake -E remove_directory pod-build
	cmake -E remove_directory build

# other (custom) targets are passed through to the cmake-generated Makefile
%::
	cmake --build pod-build $(CMAKE_CONFIG) --target $@

# Default to a less-verbose build.  If you want all the gory compiler output,
# run "make VERBOSE=1"
$(VERBOSE).SILENT:


### rules specific to this project (everything above is the generic pods makefile wrapper of a cmake project)

.PHONY: doc doxygen
doc:	doxygen doc/urdf/drakeURDF.html

doxygen :
	doxygen doc/doxyfile_matlab
	doxygen doc/doxyfile_cpp

doc/urdf/drakeURDF.html : doc/drakeURDF.xsd
ifeq ($(BUILD_SYSTEM),Darwin)
	cd doc && /Applications/oxygen/schemaDocumentationMac.sh drakeURDF.xsd -cfg:oxygen_export_settings_html.xml
else ifeq ($(OXYGEN_DIR),)
	echo "You must set the OXYGEN_DIR environment variable"
else
	cd doc && $(OXYGEN_DIR)/schemaDocumentation.sh drakeURDF.xsd -cfg:oxygen_export_settings_html.xml
endif

.PHONY: mlint
mlint	:
	matlab -nodisplay -r "addpath(fullfile(pwd,'thirdParty','runmlint')); runmlint('.mlintopts'); exit"

.PHONY: test
test	:  configure
	-@cd pod-build && ctest -D Experimental -C $(BUILD_TYPE) --output-on-failure --timeout 300

release_filelist: configure
	echo ".UNITTEST"
	echo ".mlintopts"
	find * -type f | grep -v "pod-build" | grep -v "\.valgrind" | grep -v "\.viewer-prefs" | grep -v "\.out" | grep -v "\.autosave" | grep -v "\.git" | grep -v "\.tmp" | grep -v "drake_config\.mat" | grep -v "DoxygenMatlab" | grep -v "\.aux" | grep -v "\.d" | grep -v "\.log" | grep -v "\.bib"
	find pod-build/lib -type f
	-find pod-build/bin -type f 2> /dev/null
