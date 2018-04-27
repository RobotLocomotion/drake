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

# Files covered by cpplint.
bazel query 'kind("source file", deps(attr(tags, cpplint, tests(//...))))' |
    grep -v '^@' | grep -v '/thirdParty' |
    egrep '\.(h|cc|hpp|cpp)$' |
    perl -pe 's#^//:?##g; s#:#/#g;'|
    sort > /tmp/cpplint_files.txt

echo "Files missed by cpplint() in BUILD files ..."
diff -u /tmp/cc_files.txt /tmp/cpplint_files.txt | grep "^-" || true
echo "\n"

echo "Files unknown to Bazel ..."
diff -u /tmp/git_files.txt /tmp/cc_files.txt | grep "^-" || true

