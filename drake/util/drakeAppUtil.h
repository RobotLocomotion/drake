#pragma once

#include <algorithm>
#include <string>

/*** getCommandLineOption
 * @brief Provides a platform-independent way to parse command line options
 *(getopt is not available on msvc)
 *
 * Example usage:
 * char * filename = getCommandLineOption(argv, argv + argc, "-f");
 * if (filename) { ... }
 *
 * See also commandLineOptionExists
 */
char* getCommandLineOption(char** begin, char** end,
                           const std::string& option) {
  char** itr = std::find(begin, end, option);
  if (itr != end && ++itr != end) {
    return *itr;
  }
  return 0;
}

/*** commandLineOptionExists
 * @brief Provides a platform-independent way to parse command line options
 *(getopt is not available on msvc)
 *
 * Example usage:
 * if (commandLineOptionExists(argv, argv+argc, "-h")) { ... }
 *
 * See also getCommandLineOption
 */

bool commandLineOptionExists(char** begin, char** end,
                             const std::string& option) {
  return std::find(begin, end, option) != end;
}
