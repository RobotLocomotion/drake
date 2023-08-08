import subprocess

from bazel_tools.tools.python.runfiles.runfiles import Create as CreateRunfiles

from pydrake.lcm import (
    DrakeLcm,
)


lcm = DrakeLcm()
lcm_url = lcm.get_lcm_url()
print(f"Listening for LCM messages at {lcm_url}")

# Bookkeeping for channel names.
image_channels = set()

runfiles = CreateRunfiles()
handler_bin = runfiles.Rlocation(
    "drake/bindings/pydrake/visualization/image_array_handler"
)

# Spawn up a subprocess whenever a new lcmt_image_array channel is discovered.
def on_message(channel, data):
    if channel not in image_channels:
        print(f"New channel [{channel}]. Starting a new process")
        run_args = [
            handler_bin,
            "--host=127.0.0.1",
            "--port=0",
            f"--channel={channel}"
        ]
        print(run_args)
        subprocess.Popen(run_args)
        image_channels.add(channel)
        print("Completed!")

# Below acts as the fake Meldis.
lcm.SubscribeMultichannel(
    regex="DRAKE_RGBD_CAMERA_IMAGES.*",
    handler=on_message
)

while True:
    lcm.HandleSubscriptions(timeout_millis=1000)
