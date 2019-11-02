#include "drake/common/find_loaded_library.h"

#include "drake/common/drake_throw.h"
#include "drake/common/filesystem.h"

#ifdef __APPLE__
#include <dlfcn.h>

#include <mach-o/dyld.h>
#include <mach-o/dyld_images.h>
#else  // Not __APPLE__
#include <libgen.h>  // dirname
#include <string.h>
#include <unistd.h>

#include <link.h>
#include <linux/limits.h>  // PATH_MAX
#endif

using std::string;

namespace drake {

#ifdef __APPLE__

// This code has been adapted from:
// https://stackoverflow.com/questions/4309117/determining-programmatically-what-modules-are-loaded-in-another-process-os-x/23229148#23229148

namespace {
// Reads memory from MacOS specific structures into an `unsigned char*`.
unsigned char * ReadProcessMemory(mach_vm_address_t addr,
                                  mach_msg_type_number_t* size) {
  vm_offset_t readMem;

  kern_return_t kr = vm_read(mach_task_self(), addr, *size,
                             &readMem, size);
  if (kr != KERN_SUCCESS) {
    return NULL;
  }
  return (reinterpret_cast<unsigned char *>(readMem));
}
}  // namespace
// Gets the list of all the dynamic libraries that have been loaded. Finds
// `library_name` in the list, and returns its absolute directory path.
// This function is specific to MacOS
std::optional<string> LoadedLibraryPath(const string& library_name) {
  task_dyld_info dyld_info;
  mach_msg_type_number_t count = TASK_DYLD_INFO_COUNT;
  // Getinformation from current process.
  if (task_info(mach_task_self(), TASK_DYLD_INFO,
    reinterpret_cast<task_info_t>(&dyld_info), &count) == KERN_SUCCESS) {
    // Recover list of dynamic libraries.
    mach_msg_type_number_t size = sizeof(dyld_all_image_infos);
    unsigned char* data =
      ReadProcessMemory(dyld_info.all_image_info_addr, &size);
    if (!data) {
      return std::nullopt;
    }
    dyld_all_image_infos* infos =
      reinterpret_cast<dyld_all_image_infos *>(data);

    // Recover number of dynamic libraries in list.
    mach_msg_type_number_t size2 =
      sizeof(dyld_image_info) * infos->infoArrayCount;
    unsigned char* info_addr = ReadProcessMemory(
        reinterpret_cast<mach_vm_address_t>(infos->infoArray), &size2);
    if (!info_addr) {
      return std::nullopt;
    }
    dyld_image_info* info =
      reinterpret_cast<dyld_image_info*>(info_addr);

    // Loop over the dynamic libraries until `library_name` is found.
    for (uint32_t i=0; i < infos->infoArrayCount; i++) {
      const char * pos_slash = strrchr(info[i].imageFilePath, '/');
      if (!strcmp(pos_slash + 1, library_name.c_str())) {
        // Path is always absolute on MacOS.
        return string(info[i].imageFilePath,
          pos_slash - info[i].imageFilePath);
      }
    }
  }
  return std::nullopt;
}
#else  // Not __APPLE__

// Gets the list of all the shared objects that have been loaded. Finds
// `library_name` in the list, and returns its absolute directory path.
// This function is specific to Linux.
std::optional<string> LoadedLibraryPath(const std::string& library_name) {
  void* handle = dlopen(NULL, RTLD_NOW);
  link_map *map;
  dlinfo(handle, RTLD_DI_LINKMAP, &map);
  // Loop over loaded shared objects until `library_name` is found.
  while (map) {
    // Avoid using `basename()` and `dirname()` implemented in `libgen.h`
    // because as stated in `libgen.h` documentation [1], both functions
    // may modify the content of the input c-string (which happened in the
    // original implementation and resulted in a bug).
    // [1] http://man7.org/linux/man-pages/man3/basename.3.html#DESCRIPTION
    const char* pos_slash = strrchr(map->l_name, '/');
    if (pos_slash && !strcmp(pos_slash + 1, library_name.c_str())) {
      // Check if path is relative. If so, make it absolute.
      if (map->l_name[0] != '/') {
        std::string argv0 = filesystem::read_symlink({
            "/proc/self/exe"}).string();
        return string(dirname(&argv0[0])) + "/" +
              string(map->l_name, pos_slash - map->l_name);
      } else {
        return string(map->l_name, pos_slash - map->l_name);
      }
    }
    map = map->l_next;
  }
  return std::nullopt;
}
#endif
}  // namespace drake
