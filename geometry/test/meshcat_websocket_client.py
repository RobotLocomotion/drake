# Test utility for Meshcat websockets (see the argparse for details).

import argparse
import asyncio
import json
import sys
import umsgpack
import websockets


def print_recursive_comparison(d1, d2, level='root'):
    if type(d1) != type(d2):
        print(f"{level:<20} Type mismatch")
        print(f"{level:<20}    {type(d1)}: {repr(d1)}")
        print(f"{level:<20}    {type(d2)}: {repr(d2)}")

    if isinstance(d1, dict) and isinstance(d2, dict):
        if d1.keys() != d2.keys():
            s1 = set(d1.keys())
            s2 = set(d2.keys())
            print('{:<20} + {} - {}'.format(level, s1-s2, s2-s1))
            common_keys = s1 & s2
        else:
            common_keys = set(d1.keys())

        for k in common_keys:
            print_recursive_comparison(
                d1[k], d2[k], level=f"{level}.{k}")

    elif isinstance(d1, list) and isinstance(d2, list):
        if len(d1) != len(d2):
            print(f"{level:<20} len1={len(d1)}; len2={len(d2)}")
        common_len = min(len(d1), len(d2))

        for i in range(common_len):
            print_recursive_comparison(
                d1[i], d2[i], level=f"{level}.{i}")

    else:
        if d1 != d2:
            print(f"{level:<20} {repr(d1)} != {repr(d2)}")


async def check_command(ws_url, message_number, desired_command):
    async with websockets.connect(ws_url) as websocket:
        message = ''
        for n in range(message_number):
            message = await asyncio.wait_for(websocket.recv(), timeout=10)
        command = umsgpack.unpackb(message)
        if command != desired_command:
            print("FAILED")
            print_recursive_comparison(desired_command, command)
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
