# Basic collection makefile distributed with pods version: 12.11.14

# get a list of subdirs to build by reading tobuild.txt
SUBDIRS:=$(shell grep -v "^\#" tobuild.txt)
TESTDIRS:=$(shell grep -v "^\#" totest.txt)

BUILD_SYSTEM:=$(OS)
ifeq ($(BUILD_SYSTEM),Windows_NT)
ifneq ($(CYGWIN),)# check if it's cygwin
BUILD_SYSTEM:=Cygwin
endif
else
BUILD_SYSTEM:=$(shell uname -s)
endif

# Figure out where to build the software.
#   Use BUILD_PREFIX if it was passed in.
#   If not, search up to three parent directories for a 'build' directory.
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

export BUILD_PREFIX

# build quietly by default.  For a verbose build, run "make VERBOSE=1"
$(VERBOSE).SILENT:

all:
ifeq ($(BUILD_SYSTEM), Windows_NT)
	@if not exist $(BUILD_PREFIX) ( mkdir $(BUILD_PREFIX) )
	@for %%x in ($(SUBDIRS)) do ( \
		echo ------------------------------------------- & \
		echo -- %%x & \
		echo ------------------------------------------- & \
		$(MAKE) -C %%x all || exit 2; \
	)
else
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done
endif

test: all
ifeq ($(BUILD_SYSTEM), Windows_NT)
		@if not exist $(BUILD_PREFIX) ( mkdir $(BUILD_PREFIX) )
		@for %%x in ($(SUBDIRS)) do ( \
			echo ------------------------------------------- & \
			echo -- %%x & \
			echo ------------------------------------------- & \
			$(MAKE) -C %%x test || exit 2; \
		)
else
	@for subdir in $(TESTDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir test || exit 2; \
	done
endif

clean:
ifeq ($(BUILD_SYSTEM), Windows_NT)
	@if not exist $(BUILD_PREFIX) ( mkdir $(BUILD_PREFIX) )
	@for %%x in ($(SUBDIRS)) do ( \
		echo ------------------------------------------- & \
		echo -- %%x & \
		echo ------------------------------------------- & \
		$(MAKE) -C %%x clean \
	)
else
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir clean; \
	done
endif

# other (custom) targets are passed through to the cmake-generated Makefile
%::
ifeq ($(BUILD_SYSTEM), Windows_NT)
	@if not exist $(BUILD_PREFIX) ( mkdir $(BUILD_PREFIX) )
	@for %%x in ($(SUBDIRS)) do ( \
		echo ------------------------------------------- & \
		echo -- %%x & \
		echo ------------------------------------------- & \
		$(MAKE) -C %%x $@  \
	)
else
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir $@; \
	done
endif

ifeq ($(BUILD_SYSTEM), Windows_NT)
release_filelist:
	find * -maxdepth 0 -type f | grep -v ".zip"
	find build -type f
	@for subdir in $(SUBDIRS); do \
		$(MAKE) -C $$subdir $@ | sed -e "s/^/$$subdir\//" || true; \
	done

release_zip:
	zip drake-distro `$(MAKE) release_filelist`
endif
