# Test utility for Meshcat websockets (see the argparse for details).

import argparse
import asyncio
import json
import logging
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


async def socket_operations_async(args):
    logger.info("Connecting...")
    async with websockets.connect(args.ws_url, timeout=1) as websocket:
        logger.info("... connected")
        if args.send_message is not None:
            logger.info("Sending...")
            await websocket.send(umsgpack.packb(args.send_message))
            logger.info("... sent")
        message = None
        for n in range(args.expect_num_messages):
            logger.info(f"Receiving {n+1}...")
            message = await asyncio.wait_for(websocket.recv(), timeout=10)
            logger.info("... received")
        if args.expect_message:
            parsed = umsgpack.unpackb(message)
            if parsed != args.expect_message:
                print("FAILED")
                print_recursive_comparison(parsed, args.expect_message)
                sys.exit(1)
        logger.info("Disconnecting...")
    logger.info("... disconnected")


def main():
    parser = argparse.ArgumentParser(
        description="Test utility for Meshcat websockets")
    parser.add_argument(
        "--disable-drake-valgrind-tracing", action='count',
        help="ignored by this program; see valgrind.sh for details")
    parser.add_argument(
        "--ws_url", type=str, required=True,
        help="websocket URL")
    parser.add_argument(
        "--send_message", type=json.loads, default=None, metavar="JSON",
        help="send this message (given as a json string) to the websocket")
    parser.add_argument(
        "--expect_num_messages", type=int, default=0, metavar="N",
        help="the expected number of messages to receive")
    parser.add_argument(
        "--expect_message", type=json.loads, default=None, metavar="JSON",
        help="the expected contents of the the final message "
        + "(given as a json string)")
    parser.add_argument(
        "--expect_success", type=int, default=True, metavar="0|1",
        help="must be 0 or 1; when zero, exceptions and errors are ignored")
    args = parser.parse_args()
    try:
        asyncio.get_event_loop().run_until_complete(
            socket_operations_async(args))
    except Exception as e:
        if args.expect_success:
            raise
        else:
            print(e)


assert __name__ == "__main__"
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s")
logger = logging.getLogger("meshcat_websocket_client")
main()
