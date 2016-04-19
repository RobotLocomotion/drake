#!/bin/sh

set -ex

find . -name thirdParty -prune -o -name pod-build -prune -o -name 'doc' -prune -o -name '*.h' -print0 |
  xargs --verbose -n1 -0 perl -ne '
BEGIN { undef $/; }

if (m|^(//[^\n]*\n)*\n*#ifndef (DRAKE_\S+)|) {
  $guard = $2;
  print STDERR "Found $guard\n";
  s|#ifndef\s+$guard\s*\n#define\s+$guard\s*|#pragma once|m or die("cannot fix open");
  s|#endif *([/*]*\s*$guard\s*[/*]*)?\s*\n$|| or die("cannot fix close");
} else {
  die("Cannot find include guard");
}
'
