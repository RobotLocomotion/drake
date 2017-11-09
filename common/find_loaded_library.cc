#include "drake/common/find_loaded_library.h"

#ifdef __APPLE__
#include <dlfcn.h>

#include <mach-o/dyld.h>
#include <mach-o/dyld_images.h>
#else  // Not __APPLE__
#include <libgen.h>
#include <string.h>

#include <link.h>
#endif

using std::string;

namespace drake {

#ifdef __APPLE__

// This code has been adapted from:
// https://stackoverflow.com/questions/4309117/determining-programmatically-what-modules-are-loaded-in-another-process-os-x/23229148#23229148

// Reads memory from MacOS specific structures into an `unsigned char*`.
unsigned char * readProcessMemory(mach_vm_address_t addr,
                                  mach_msg_type_number_t* size) {
  mach_msg_type_number_t  dataCnt =
    reinterpret_cast<mach_msg_type_number_t>(*size);
  vm_offset_t readMem;

  kern_return_t kr = vm_read(mach_task_self(), addr, *size,
                             &readMem, &dataCnt);
  if (kr) {
    return NULL;
  }
  return ( reinterpret_cast<unsigned char *>(readMem));
}

// Gets the list of all the dynamic libraries that have been loaded. Finds
// `library_name` in the list, and returns its directory path appended
// with relative directory to find resource files in drake install tree.
// This function is specific to MacOS
optional<string> loaded_library_path(const std::string &library_name) {
  optional<string> binary_dirname;
  struct task_dyld_info dyld_info;
  mach_msg_type_number_t count = TASK_DYLD_INFO_COUNT;
  // Getinformation from current process.
  if (task_info(mach_task_self(), TASK_DYLD_INFO,
    reinterpret_cast<task_info_t>(&dyld_info), &count) == KERN_SUCCESS) {
    // Recover list of dynamic libraries.
    mach_msg_type_number_t size = sizeof(struct dyld_all_image_infos);
    uint8_t* data =
      readProcessMemory(dyld_info.all_image_info_addr, &size);
    if (!data) {
      return binary_dirname;
    }
    struct dyld_all_image_infos* infos =
      reinterpret_cast<struct dyld_all_image_infos *>(data);

    // Recover number of dynamic libraries in list.
    mach_msg_type_number_t size2 =
      sizeof(struct dyld_image_info) * infos->infoArrayCount;
    uint8_t* info_addr = readProcessMemory(
        reinterpret_cast<mach_vm_address_t>(infos->infoArray), &size2);
    if (!info_addr) {
      return binary_dirname;
    }
    struct dyld_image_info* info =
      reinterpret_cast<struct dyld_image_info*>(info_addr);

    // Loop over the dynamic libraries until `library_name` is found.
    for (uint32_t i=0; i < infos->infoArrayCount; i++) {
      const char * pos_slash = strrchr(info[i].imageFilePath, '/');
      if (!strcmp(pos_slash + 1, library_name.c_str())) {
        binary_dirname = string(info[i].imageFilePath,
          pos_slash - info[i].imageFilePath);
        break;
      }
    }
  }
  return binary_dirname;
}
#else  // Not __APPLE__

// This code has been adapted from:
// http://syprog.blogspot.ru/2011/12/listing-loaded-shared-objects-in-linux.html

// Chained list of shared objects
struct lmap {
  void*    base_address;     // Base address of the shared object
  char*    path;             // Absolute file name (path) of the shared object
  void*    not_needed;       // Pointer to the dynamic section of the SO
  struct lmap *next, *prev;  // chain of loaded objects
};

// Content of that dlopen is saved in this structure.
struct something {
  void*  pointers[3];
  struct something* ptr;
};

// Gets the list of all the shared objects that have been loaded. Finds
// `library_name` in the list, and returns its directory path appended
// with relative directory to find resource files in drake install tree.
// This function is specific to Linux.
optional<string> loaded_library_path(const std::string &library_name) {
  optional<string> binary_dirname;
  struct lmap* pl;
  void* ph = dlopen(NULL, RTLD_NOW);
  struct something* p = reinterpret_cast<struct something*>(ph);
  p = p->ptr;
  pl = reinterpret_cast<struct lmap*>(p->ptr);
  // Loop over loaded shared objects until `library_name` is found.
  while (NULL != pl) {
    if (!strcmp(basename(pl->path), library_name.c_str())) {
      binary_dirname = string(dirname(pl->path));
      break;
    }
    pl = pl->next;
  }
  return binary_dirname;
}
#endif
}  // namespace drake
