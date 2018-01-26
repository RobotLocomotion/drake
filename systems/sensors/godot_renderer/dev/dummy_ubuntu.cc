#include <core/register_core_types.h>
#include <drivers/gles3/rasterizer_gles3.h>
#include <drivers/register_driver_types.h>
#include <drivers/unix/dir_access_unix.h>
#include <drivers/unix/file_access_unix.h>
#include <os/thread_dummy.h>
#include <servers/visual/visual_server_raster.h>

// This is some small sample code that uses a few things from Godot, to show
// that we can link without errors.  Once we have more real code using Godot,
// we should remove this dummy program.  (Note that this program more or less
// always segfaults at the moment.)
int main() {
  RID_OwnerBase::init_rid();
  ThreadDummy::make_default();
  SemaphoreDummy::make_default();
  MutexDummy::make_default();
  FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_RESOURCES);
  FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_USERDATA);
  FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_FILESYSTEM);
  DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_RESOURCES);
  DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_USERDATA);
  DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_FILESYSTEM);
  ClassDB::init();
  register_core_types();
  register_core_driver_types();
  RasterizerGLES3::register_config();
  RasterizerGLES3::make_current();
  auto visual_server = memnew(VisualServerRaster);
  visual_server->init();

  return 0;
}
