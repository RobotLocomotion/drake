# Usage:
# python3 meshcat_websocket_client.py ws_url message_number desired_command_json  # noqa
#
# This test script connects to a websocket on localhost at `port`, listens for
# `message_number` received messages, unpacks the last message and compares it
# to the dictionary passed via `desired_command_json`.

import asyncio
import json
import sys
import umsgpack
import websockets


async def CheckCommand(ws_url, message_number, desired_command):
    async with websockets.connect(ws_url) as websocket:
        message = ''
        for n in range(message_number):
            message = await asyncio.wait_for(websocket.recv(), timeout=10)
        command = umsgpack.unpackb(message)
        if command != desired_command:
            print("FAILED")
            print(f"Expected: {desired_command}")
            print(f"Received: {command}")
            exit(1)

if len(sys.argv) != 4:
    print(sys.argv)
    print("Usage: python3 meshcat_websocket_client.py ws_url "
          + "message_number desired_command_json")
    exit(1)

ws_url = sys.argv[1]
message_number = int(sys.argv[2])
desired_command_json = sys.argv[3]
desired_command = json.loads(desired_command_json)

asyncio.get_event_loop().run_until_complete(
    CheckCommand(ws_url, message_number, desired_command))

exit(0)
