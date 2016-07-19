from ctypes.util import find_library
import os
import subprocess
from sys import argv

def call_otool(lib_name):
    otool_proc = subprocess.Popen(['otool', '-L', lib_name], stdout=subprocess.PIPE)
    return otool_proc.stdout

def main():
    root_dir = argv[1]
    lib_link_to_search_for = argv[2]
    
    libs_ref_bot2_core = []

    if "build" in root_dir:
        root_dir = os.path.split(root_dir)[0]
        for dir_name, _, files in os.walk(root_dir):
            libraries = [lib_file for lib_file in files if (lib_file.endswith(".a") or lib_file.endswith(".dylib"))]
            if libraries:
                for library in libraries:
                    otool_output = call_otool(os.path.join(dir_name, library))
                    for otool_lin in otool_output.readlines():
                        if lib_link_to_search_for in str(otool_lin):
                            print "Found %s in %s.\n" % (lib_link_to_search_for, library)

if __name__ == '__main__':
    main()


    
