#!/bin/bash

set -e

# Files in git.
git ls-files |
    egrep '\.(h|cc|hpp|cpp)$' |
    sort > /tmp/git_files.txt

# Files covered by cc_ something.
bazel query 'kind("source file", deps(kind("cc_.* rule", //...)))' |
    grep -v '^@' | grep -v '/thirdParty' |
    egrep '\.(h|cc|hpp|cpp)$' |
    perl -pe 's#^//:?##g; s#:#/#g;' |
    sort > /tmp/cc_files.txt

echo "Files unknown to Bazel ..."
diff -u /tmp/git_files.txt /tmp/cc_files.txt | grep "^-" || true
