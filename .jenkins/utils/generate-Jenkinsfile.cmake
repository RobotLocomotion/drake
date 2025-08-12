# This script exists to generate a Jenkinsfile in drake/.jenkins/ from a
# template in drake/.jenkins/templates/. It should be called from
# drake/.jenkins/ as:
#   cmake -P utils/generate-Jenkinsfile.cmake <config>
# where <config> is one of:
#   * experimental
#   * external-examples
#   * production
#   * staging
# Maintainers should run this script to re-generate the Jenkinsfiles anytime
# the Jenkinsfile templates or the utils change.

# Parse arguments.
if (NOT CMAKE_ARGC EQUAL 4)
  message(FATAL_ERROR "This script expects a single argument.")
endif()
set(JENKINS_CONFIG "${CMAKE_ARGV3}")

# Load utils.groovy.
set(UTILS_FILE "${CMAKE_CURRENT_SOURCE_DIR}/utils/utils.groovy")
file(READ ${UTILS_FILE} UTILS_CONTENT)

# Generate the output file.
set(JENKINSFILE_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/templates/Jenkinsfile-${JENKINS_CONFIG}.in")
set(JENKINSFILE_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/Jenkinsfile-${JENKINS_CONFIG}")
if (NOT EXISTS ${JENKINSFILE_TEMPLATE})
  message(FATAL_ERROR "Failed to find template file ${JENKINSFILE_TEMPLATE}.")
endif()
configure_file(${JENKINSFILE_TEMPLATE} ${JENKINSFILE_OUTPUT} @ONLY)
