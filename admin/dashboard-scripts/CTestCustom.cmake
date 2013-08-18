set(CTEST_CUSTOM_MAXIMUM_NUMBER_OF_ERRORS   5000)
set(CTEST_CUSTOM_MAXIMUM_NUMBER_OF_WARNINGS 5000)


set(CTEST_CUSTOM_COVERAGE_EXCLUDE
  ${CTEST_CUSTOM_COVERAGE_EXCLUDE}
  ".*/software/externals/*"
  )


set(CTEST_CUSTOM_ERROR_EXCEPTION
  ${CTEST_CUSTOM_ERROR_EXCEPTION}

  "error: could not create '/usr/local/lib/python2.7/dist-packages/pyflann': Permission denied"
  ".*configure.ac.*installing.*missing.*" # printed during configure in camunits

  )


set(CTEST_CUSTOM_WARNING_EXCEPTION
  ${CTEST_CUSTOM_WARNING_EXCEPTION}

  "warning: 'SHOT' is deprecated" # from include/pcl-1.6/pcl/point_representation.h
  "jN forced in submake.*disabling jobserver mode."

  "software/externals/opencv-drc/"
  "software/externals/pcl_drc/"
  "software/externals/fovis-git/"
  "software/externals/bullet/"
  "software/build/include/"
  "openni/XnCppWrapper.h"
  )
