# This script exists to generate Jenkinsfiles in drake/.jenkins/ from the
# templates in drake/.jenkins/templates/. It should be called from
# drake/.jenkins/ as:
#   cmake -P utils/generate-Jenkinsfiles.cmake
# Maintainers should run this script to re-generate the Jenkinsfiles anytime
# the Jenkinsfile templates or the utils change.

# Load utils.groovy.
set(UTILS_FILE "${CMAKE_CURRENT_SOURCE_DIR}/utils/utils.groovy")
file(READ ${UTILS_FILE} UTILS_CONTENT)

# Define configurations. This list should match the Jenkinsfiles themselves,
# as well as all of the files in templates/, and the README.
set(JENKINS_CONFIGS
  "experimental"
  "external-examples"
  "production"
  "staging"
)

# Generate the output files.
foreach(config IN LISTS JENKINS_CONFIGS)
  set(JENKINSFILE_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/templates/Jenkinsfile-${config}.in")
  set(JENKINSFILE_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/Jenkinsfile-${config}")
  if (NOT EXISTS ${JENKINSFILE_TEMPLATE})
    message(FATAL_ERROR "Failed to find template file ${JENKINSFILE_TEMPLATE}.")
  endif()
  configure_file(${JENKINSFILE_TEMPLATE} ${JENKINSFILE_OUTPUT} @ONLY)
endforeach()
