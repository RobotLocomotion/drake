s|.*<version>\(\([0-9]*\)\.\([0-9]*\)\.\([0-9]*\)\)</version>.*|set(FCL_VERSION \1)\
set(FCL_MAJOR_VERSION \2)\
set(FCL_MINOR_VERSION \3)\
set(FCL_PATCH_VERSION \4)|p
