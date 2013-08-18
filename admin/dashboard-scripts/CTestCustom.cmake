set(CTEST_CUSTOM_MAXIMUM_NUMBER_OF_ERRORS   5000)
set(CTEST_CUSTOM_MAXIMUM_NUMBER_OF_WARNINGS 5000)


set(CTEST_CUSTOM_COVERAGE_EXCLUDE
  ${CTEST_CUSTOM_COVERAGE_EXCLUDE}

  # include any directories you want to exclude from coverage testing

  )


set(CTEST_CUSTOM_ERROR_EXCEPTION
  ${CTEST_CUSTOM_ERROR_EXCEPTION}

  # include any error strings you want to ignore 

  )


set(CTEST_CUSTOM_WARNING_EXCEPTION
  ${CTEST_CUSTOM_WARNING_EXCEPTION}

  # include any strings or directories you want to ignore warnings in/for

  )
