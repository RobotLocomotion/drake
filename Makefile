
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

# extra logic to support complex CMAKE_FLAG inputs (e.g. passed in from CMakeLists.txt) which might have quotes, etc.
ifeq ($(BUILD_SYSTEM),"Windows_NT")
  CMAKE_FLAGS=$(strip $(shell echo off & setlocal enableextensions enabledelayedexpansion\
				set /a "count = 1"\
				:while1\
				if defined CMAKE_FLAGS%count% (\
				  for /F  %%a in ('echo CMAKE_FLAGS%count%') do set CMAKE_FLAGS=%CMAKE_FLAGS% !%%a!\
				  set /a "count = count + 1"\
				  goto :while1\
				)\
				if defined CMAKE_FLAGS  echo %CMAKE_FLAGS% ))
else
	CMAKE_FLAGS=$(strip $(shell count=1; eval flag=\$$CMAKE_FLAGS$$count; while [ ! -z "$$flag" ]; do CMAKE_FLAGS="$$CMAKE_FLAGS $$flag"; count=`expr $$count + 1`; eval flag=\$$CMAKE_FLAGS$$count; done; echo $$CMAKE_FLAGS ))
endif

.PHONY: all
all: pod-build/Makefile
#	cmake --build pod-build --config $(BUILD_TYPE)

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
	@echo BUILD_SYSTEM = $(BUILD_SYSTEM)
	@echo "Configuring with CMAKE_FLAGS: $(CMAKE_FLAGS)"
	@cd pod-build && cmake -L $(CMAKE_FLAGS) -DCMAKE_INSTALL_PREFIX=$(BUILD_PREFIX) \
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
