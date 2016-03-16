cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(VERSION 2.6)

project(tinydir C)

INCLUDE_DIRECTORIES(..)

################################
# Add definitions

if(MSVC)
	add_definitions(-W4 -WX)
else()
	add_definitions(-fsigned-char -Wall -W -Wshadow -Wstrict-prototypes -Wpointer-arith -Wcast-qual -Winline -Werror)
endif()

################################
# Add targets
add_executable(file_open_sample file_open_sample.c)
add_executable(iterate_sample iterate_sample.c)
add_executable(random_access_sample random_access_sample.c)
add_executable(interactive_sample interactive_sample.c)
