#!/usr/bin/env python

"""
@file
This tests basic FIFO functionality with Python.
This is done because there appear to be errors on CI that I (eric.cousineau)
cannot reproduce on my machine.
"""

from __future__ import print_function
import os
import subprocess
import sys
from threading import Thread, Event
import time

filepath = "/tmp/fifo_test"
if os.path.exists(filepath):
    os.remove(filepath)
subprocess.check_output("mkfifo {}".format(filepath), shell=True)

# Notifications from the given threads.
writer_event = Event()
reader_event = Event()
# Success metrics from the given threads.
writer_success = False
reader_success = False
# Logistics.
writer_wait = 0.1
token_1 = "abcd"
token_2 = "x" * 100


def wait(e):
    e.wait()
    e.clear()


def writer_loop():
    global writer_success
    writer_event.set()  # Open for reading.
    wait(reader_event)  # Start timing for open.
    # Wait for a bit.
    time.sleep(writer_wait)
    # Open FIFO for writing.
    f = open(filepath, 'wb')
    # Write some info.
    f.write(token_1)
    f.flush()
    wait(reader_event)  # After first token read.
    # Write final token (while reader is waiting) and close file.
    f.write(token_2)
    f.close()
    time.sleep(writer_wait)
    writer_event.set()  # Close file.
    print("[ - Writer Done ]")
    writer_success = True


def reader_loop():
    global reader_success
    # Wait for writer loop to start before we begin timing.
    wait(writer_event)  # Open for reading.
    # Open FIFO for reading.
    start = time.time()
    reader_event.set()  # Start timing for open.
    # Ensure that opening for read blocks until it is open for writing.
    f = open(filepath, 'rb')
    elapsed = time.time() - start
    assert elapsed >= writer_wait, "{} < {}".format(elapsed, writer_wait)
    # Expect the basics.
    assert f.read(len(token_1)) == token_1
    reader_event.set()  # Read first token.
    wait(writer_event)  # Close file.
    # Read.
    assert f.read(len(token_2)) == token_2
    print("[ - Reader Done ]")
    reader_success = True


writer = Thread(target=writer_loop)
reader = Thread(target=reader_loop)
writer.start()
reader.start()
writer.join()
reader.join()
print("[ Done ]")

if writer_success and reader_success:
    exit(0)
else:
    sys.stderr.write("Error occurred!\n")
    exit(1)
