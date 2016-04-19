#!/bin/sh

set -ex

find . -name thirdParty -prune -o -name pod-build -prune -o -name 'doc' -prune -o -name '*.h' -print0 |
  xargs -n1 -0 perl -pi -e '
BEGIN { undef $/; }

if (m|^(//[^\n]*\n)*\n*#ifndef (DRAKE_\S+)|) {
  $guard = $2;
  s|#ifndef\s+$guard\s*\n#define\s+$guard\s*\n*|#pragma once\n\n|m or die("cannot fix open");
  s|\n*#endif *([/*]*\s*$guard\s*[/*]*)?\s*\n$|\n| or die("cannot fix close");
} else {
  die("Cannot find include guard");
}
'
