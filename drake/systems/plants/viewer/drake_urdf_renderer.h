#ifndef __drake_urdf_renderer_h__
#define __drake_urdf_renderer_h__

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>

#ifdef __cplusplus
extern "C" {
#endif

void drake_urdf_add_renderer_to_viewer(BotViewer* viewer, lcm_t* lcm, int priority);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
