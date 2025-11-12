# Test utility for Meshcat websockets (see the argparse for details).

import argparse
import asyncio
import json
import logging
import sys

import umsgpack
import websockets

# BEGIN ugly hack
#
# https://bugs.launchpad.net/ubuntu/+source/python-websockets/+bug/1969902
#
# Python 3.10 (Jammy) removed the `loop` parameter from various asyncio APIs.
# Unfortunately, Jammy shipped with python3-websokets 9.1, which is still
# passing a `loop` parameter, resulting in asyncio expressing its displeasure
# by means of exceptions. Trying to patch websokets directly is not reasonable.
# Instead, patch the asyncio functions to remove the `loop` parameter.
#
# TODO(mwoehlke-kitware): Remove this when Jammy's python3-websockets has been
# updated to 10.0 or later.
_asyncio_lock_ctor = asyncio.Lock.__init__
_asyncio_wait_for = asyncio.wait_for
_asyncio_sleep = asyncio.sleep


def _patch_asyncio(orig):
    def _patched(*args, **kwargs):
        if "loop" in kwargs and kwargs["loop"] is None:
            kwargs.pop("loop")
        return orig(*args, **kwargs)

    return _patched


asyncio.Lock.__init__ = _patch_asyncio(_asyncio_lock_ctor)
asyncio.wait_for = _patch_asyncio(_asyncio_wait_for)
asyncio.sleep = _patch_asyncio(_asyncio_sleep)

# END ugly hack

# https://bugs.launchpad.net/ubuntu/+source/u-msgpack-python/+bug/1979549
#
# Jammy shipped with python3-u-msgpack 2.3.0, which tries to use
# `collections.Hashable`, which was removed in Python 3.10. Work around this by
# monkey-patching `Hashable` into `umsgpack.collections`.
#
# TODO(mwoehlke-kitware): Remove this when Jammy's python3-u-msgpack has been
# updated to 2.5.2 or later.
if not hasattr(umsgpack, "Hashable"):
    import collections

    setattr(umsgpack.collections, "Hashable", collections.abc.Hashable)


def print_recursive_comparison(d1, d2, level="root"):
    if type(d1) is not type(d2):
        print(f"{level:<20} Type mismatch")
        print(f"{level:<20}    {type(d1)}: {repr(d1)}")
        print(f"{level:<20}    {type(d2)}: {repr(d2)}")

    if isinstance(d1, dict) and isinstance(d2, dict):
        if d1.keys() != d2.keys():
            s1 = set(d1.keys())
            s2 = set(d2.keys())
            print("{:<20} + {} - {}".format(level, s1 - s2, s2 - s1))
            common_keys = s1 & s2
        else:
            common_keys = set(d1.keys())

        for k in common_keys:
            print_recursive_comparison(d1[k], d2[k], level=f"{level}.{k}")

    elif isinstance(d1, list) and isinstance(d2, list):
        if len(d1) != len(d2):
            print(f"{level:<20} len1={len(d1)}; len2={len(d2)}")
        common_len = min(len(d1), len(d2))

        for i in range(common_len):
            print_recursive_comparison(d1[i], d2[i], level=f"{level}.{i}")

    else:
        if d1 != d2:
            print(f"{level:<20} {repr(d1)} != {repr(d2)}")


async def socket_operations_async(args):
    logger.info("Connecting...")
    async with websockets.connect(args.ws_url, close_timeout=1) as websocket:
        logger.info("... connected")
        if args.send_message is not None:
            logger.info("Sending...")
            await websocket.send(umsgpack.packb(args.send_message))
            logger.info("... sent")
        message = None
        for n in range(args.expect_num_messages):
            logger.info(f"Receiving {n + 1}...")
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
        description="Test utility for Meshcat websockets"
    )
    parser.add_argument(
        "--disable-drake-valgrind-tracing",
        action="count",
        help="ignored by this program; see valgrind.sh for details",
    )
    parser.add_argument(
        "--ws_url", type=str, required=True, help="websocket URL"
    )
    parser.add_argument(
        "--send_message",
        type=json.loads,
        default=None,
        metavar="JSON",
        help="send this message (given as a json string) to the websocket",
    )
    parser.add_argument(
        "--expect_num_messages",
        type=int,
        default=0,
        metavar="N",
        help="the expected number of messages to receive",
    )
    parser.add_argument(
        "--expect_message",
        type=json.loads,
        default=None,
        metavar="JSON",
        help="the expected contents of the the final message "
        + "(given as a json string)",
    )
    parser.add_argument(
        "--expect_success",
        type=int,
        default=True,
        metavar="0|1",
        help="must be 0 or 1; when zero, exceptions and errors are ignored",
    )
    args = parser.parse_args()
    try:
        asyncio.run(socket_operations_async(args))
    except Exception as e:
        if args.expect_success:
            raise
        else:
            print(e)


assert __name__ == "__main__"
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s",
)
logger = logging.getLogger("meshcat_websocket_client")
main()
