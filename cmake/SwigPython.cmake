include(CMakeParseArguments)

function(add_swig_python_module)
	# Parse our arguments and make sure we got the required ones
	set(options CPLUSPLUS)
	set(oneValueArgs SWIG_I_FILE TARGET )
	set(multiValueArgs INCLUDE_DIRS LINK_LIBRARIES SWIG_INCLUDE_DIRS DESTINATION)
	cmake_parse_arguments(swigpy "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
	if (NOT swigpy_TARGET)
		message(FATAL_ERROR "Error using add_swig_python_module: Please provide a target name with 'TARGET targetname'")
	endif()
	if (NOT swigpy_SWIG_I_FILE)
		message(FATAL_ERROR "Error using add_swig_python_module: Please provide the full path to your .i file with 'SWIG_I_FILE filepath'")
	endif()

	# Find python and get its version number
	find_package(PythonInterp REQUIRED)
	set(PYVERSION "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

	if(APPLE)
		# Overload the PYTHON_INCLUDE_DIR and PYTHON_LIBRARY because, on OSX with a homebrew-provided python, cmake latches on to an old Apple-provided python install. 
		execute_process(COMMAND python${PYTHON_VERSION_MAJOR}-config --prefix
			OUTPUT_VARIABLE PYTHON_PREFIX
			OUTPUT_STRIP_TRAILING_WHITESPACE
			)
		if (PYTHON_VERSION_MAJOR GREATER 2)
			set(PYTHON_INCLUDE_DIR ${PYTHON_PREFIX}/include/python${PYVERSION}m)
		else()
			set(PYTHON_INCLUDE_DIR ${PYTHON_PREFIX}/include/python${PYVERSION})
		endif()

		set(PYTHON_LIBRARY ${PYTHON_PREFIX}/lib/libpython${PYVERSION}.dylib)
		# These variable settings will affect the behavior of find_package(PythonLibs)
	endif()

	# Load the swig macros
	find_package(SWIG REQUIRED)
	include(UseSWIG)

	# Find the numpy header paths and include them. This calls the FindNumPy.cmake file included in this repo. 
	find_package(NumPy REQUIRED)
	include_directories(${NUMPY_INCLUDE_DIRS})

	# Find the python libraries so that we can link against them.
	find_package( PythonLibs REQUIRED )
	include_directories( ${PYTHON_INCLUDE_DIRS} )

	# Include any source directories that swig will need to find our c++ header files
	foreach(dir IN LISTS swigpy_INCLUDE_DIRS)
		include_directories(${dir})
	endforeach(dir)

	# Tell SWIG that we're compiling a c++ (not c) file, and tell it to use python3 if appropriate. 
	if (swigpy_CPLUSPLUS)
		set(CPLUSPLUS ON)
	else()
		set(CPLUSPLUS OFF)
	endif()
	if (PYTHON_VERSION_MAJOR GREATER 2)
		set_source_files_properties(${swigpy_SWIG_I_FILE} PROPERTIES
			CPLUSPLUS ${CPLUSPLUS}
			SWIG_FLAGS "-py3"
			)
	else()
		set_source_files_properties(${swigpy_SWIG_I_FILE} PROPERTIES 
			CPLUSPLUS ${CPLUSPLUS})
	endif()

	# Tell swig to also look for .i interface files in these folders
	foreach(dir IN LISTS swigpy_SWIG_INCLUDE_DIRS)
		set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-I${dir}")
	endforeach(dir)

	# Tell swig to build python bindings for our target library and link them against the C++ library. 
	swig_add_module(${swigpy_TARGET} python ${swigpy_SWIG_I_FILE})
	swig_link_libraries(${swigpy_TARGET} ${swigpy_LINK_LIBRARIES} ${PYTHON_LIBRARIES})

	# Set a variable in the scope of the cmake file that called this function so that it can be referenced later (for example, to 
	set(SWIG_MODULE_${swigpy_TARGET}_REAL_NAME ${SWIG_MODULE_${swigpy_TARGET}_REAL_NAME} PARENT_SCOPE)

	foreach(dir IN LISTS swigpy_DESTINATION)
		message("installing to: " ${dir})
		install(TARGETS ${SWIG_MODULE_${swigpy_TARGET}_REAL_NAME} DESTINATION ${dir})
		install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${swigpy_TARGET}.py DESTINATION ${dir})
	endforeach(dir)
endfunction()
