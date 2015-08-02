
#include <bot_lcmgl_client/lcmgl.h>

#define UNUSED(x) (void)(x)

int main(int argc, char* argv[])
{
  UNUSED(argc);
  UNUSED(argv);

  lcm_t * lcm;
  lcm = lcm_create(NULL);

  bot_lcmgl_t* lcmgl = bot_lcmgl_init(lcm, "lcmgl-example");

//  bot_lcmgl_color3f(lcmgl, 0, 0, 1);  // blue

  double xyz[3] = {0.0, 0.0, 0.0};
  bot_lcmgl_sphere(lcmgl, xyz, .1, 36, 36);

  bot_lcmgl_switch_buffer(lcmgl);

  bot_lcmgl_destroy(lcmgl);

  return 0;
}
