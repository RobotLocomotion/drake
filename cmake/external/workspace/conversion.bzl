# This file contains some helper functions that are meant to be used with
# CMake imported targets.

def split_cmake_list(cmake_list_str):
    """Converts a string containing a CMake-style list into a 'proper' list.
    In particular, empty strings remain empty `[]`, not 1-element lists `[""]`.
    """
    if len(cmake_list_str) == 0:
        return []
    return cmake_list_str.split(";")
