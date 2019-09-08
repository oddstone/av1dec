#!/usr/bin/env python3

# Copyright © 2019, av1dec authors
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import sys
import os
import hashlib
from os.path import dirname, join, realpath, basename
import re

def get_md5(path):
    h = hashlib.md5()
    print(path)
    with open(path, 'rb') as f:
        while True:
            data = f.read(64 * 1024)
            if not data:
                break
            h.update(data)
    return h.hexdigest()

def get_ref_md5(fn):
    dir = dirname(fn)
    name = basename(fn)
    bits = os.path.join(dir, "bits.md5")
    if not os.path.exists(bits):
        return None
    with open(bits) as f:
        suffix = "  " + name
        for line in f.readlines():
            line = line.strip()
            #print("+" + line + "-")
            if line.endswith(suffix):
                line = line.replace(suffix, "")
                return line
    return None

def decode(input, output):
    av1dec = join(dirname(realpath(__file__)), "../build/tests/Debug/av1dec.exe")
    if os.path.exists(output):
        os.remove(output)
    cmd = av1dec + " -i " + input + " " + output
    print(cmd)
    os.system(cmd)

PASSED = 0
FAILED = 1
SKIPPED = 2
def test(f, log = False):
    yuv = "test.yuv"
    decode(f, yuv)
    if not os.path.exists(yuv):
        print("decode failed for "+f)
        return FAILED
    md5 = get_md5(yuv)
    refmd5 = get_ref_md5(f)
    if not refmd5:
        print(basename(f) + " has no ref md5")
        return SKIPPED
    if refmd5 == md5:
        return PASSED
    print("md5 mismatch ref = " + refmd5 + " md5 = "+md5)
    return FAILED

def is_candidiate(f):
    filename, ext = os.path.splitext(f)
    ext = ext.lower()
    supported = [
        ".ivf"
    ]
    return ext in supported

def print_summary(name, files):
    if len(files) > 0:
        print(name + " files:")
        for f in files:
            print("    " + f)

argc = len(sys.argv)
if argc != 2 and argc != 1:
    print("usage: " + sys.argv[0] + " directory/file")
    sys.exit(1)

if argc == 2:
    path = sys.argv[1]
else:
    path = av1dec = join(dirname(realpath(__file__)), "..", "bits")
#test file
if os.path.isfile(path):
    pss = test(path, True)
    print(basename(path) + " passed" if pss  else " failed")
    sys.exit(0)

status = [0, 0, 0]
summary = [[], [], []]
for root, dirs, files in os.walk(path):
    for f in files:
        fn = join(root, f)
        #print("root = "+root)
        #print("f = "+f)
        if is_candidiate(fn):
            s = test(fn)
            status[s] += 1
            summary[s].append(f)
print("")
print("+++++++++ report +++++++++");
print_summary("failed", summary[FAILED])
print_summary("skipped", summary[SKIPPED])
print_summary("passed", summary[PASSED])
print("")
print("total = "+ str(status[PASSED] + status[FAILED] + status[SKIPPED]) +", failed = "+ str(status[FAILED]) + ", skipped = " + str(status[SKIPPED]))
print("----------")
