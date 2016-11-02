#include <lcm/lcm.h>
#include <bot_lcmgl_client/lcmgl.h>

// NOLINTNEXTLINE(build/include) Generated C code (N.B. not C++) is odd.
#include "drake_lcmt_zmp_com_observer_state.h"

bot_lcmgl_t *lcmgl;

static void handleMessage(const lcm_recv_buf_t *rbuf, const char *channel,
                          const drake_lcmt_zmp_com_observer_state *msg,
                          void *user) {
  double height = msg->ground_plane_height + 0.01;  // a little above the ground
                                                    // to make sure it doesn't
                                                    // get occluded by the
                                                    // ground plane.
  bot_lcmgl_color3f(lcmgl, 1.0, 0.0, 0.0);          // red
  double xyz[] = {msg->com[0], msg->com[1], height};
  bot_lcmgl_sphere(lcmgl, xyz, .01, 36, 36);
  bot_lcmgl_switch_buffer(lcmgl);
}

int main(int argc, char **argv) {
  lcm_t *lcm;
  lcm = lcm_create(nullptr);
  if (!lcm) return 1;

  lcmgl = bot_lcmgl_init(lcm, "zmp-based CoM estimate");
  drake_lcmt_zmp_com_observer_state_subscribe(lcm, "ZMP_COM_OBSERVER_STATE",
                                              &handleMessage, nullptr);

  while (true) lcm_handle(lcm);

  bot_lcmgl_destroy(lcmgl);
  lcm_destroy(lcm);

  return 0;
}
