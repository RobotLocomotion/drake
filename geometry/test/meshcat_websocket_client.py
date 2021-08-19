# Test utility for Meshcat websockets (see the argparse for details).

import argparse
import asyncio
import json
import sys
import umsgpack
import websockets


async def check_command(ws_url, message_number, desired_command):
    async with websockets.connect(ws_url) as websocket:
        message = ''
        for n in range(message_number):
            message = await asyncio.wait_for(websocket.recv(), timeout=10)
        command = umsgpack.unpackb(message)
        if command != desired_command:
            print("FAILED")
            print(f"Expected: {desired_command}")
            print(f"Received: {command}")
            sys.exit(1)


assert __name__ == "__main__"

parser = argparse.ArgumentParser(
    description="Test utility for Meshcat websockets")
parser.add_argument("ws_url", type=str, help="websocket URL")
parser.add_argument(
    "message_number",
    type=int,
    help="number of messages to receive")
parser.add_argument(
    "desired_command_json",
    type=str,
    help="the last message will be unpacked (to a dictionary) and compared to "
    + "this desired value (specified as a json string)"
)
args = parser.parse_args()

asyncio.get_event_loop().run_until_complete(
    check_command(args.ws_url, args.message_number,
                  json.loads(args.desired_command_json)))

sys.exit(0)
