#!/usr/bin/env bash
# This is a local Git pre-commit hook to enforce the following:
#
# - Disallow submodule link updates that rewind the submodule.

die() {
  echo 'pre-commit hook failure' 1>&2
  echo '-----------------------' 1>&2
  echo '' 1>&2
  echo "$@" 1>&2
  exit 1
}

short_commit() {
  git rev-parse --short "$1" 2>/dev/null || echo "$1"
}

index_diffs() {
  git diff-index --cached "$1" -- |
  sed -n '/^:[^:]/ {s/^://;p;}'
}

# Match diffs that add or change a path mode to the special
# value that Git uses to reference submodule commit objects.
filter_diffs_module() {
  grep '^...... 160000'
}

# Visit the submodule's `.git` directory and compute the most recent
# common ancestor ("merge base") between the old and new commits
# referenced by this submodule link update.  If the new value (dst_obj)
# is the common ancestor then we know that it has been rewound from
# the old value (src_obj).  This is likely not intentional.
check_module_rewind() {
  readonly parent_name="$1"

  # Check if this is a rewind in the submodule's history.
  readonly base="$(GIT_DIR="$file/.git" \
                   git merge-base "$src_obj" "$dst_obj" 2>/dev/null)" || base=''
  if test "$base" != "$dst_obj"; then
    return
  fi

  # It is a rewind.  Check for overriding configuration.
  readonly src_short="$(GIT_DIR="$file/.git" short_commit "$src_obj")"
  readonly dst_short="$(GIT_DIR="$file/.git" short_commit "$dst_obj")"
  readonly override="$src_short..$dst_short"
  if update="$(git config --get hooks."$file".update)" &&
     test "{$update}" = "{$override}"; then
    return
  fi

  # Produce a diagnostic.
  readonly parent_short="$(short_commit "$parent_name")"
  echo 'This commit would rewind a submodule link:

  "'"$file"'"  '"$src_short => $dst_short"'

from the newer version in '"$parent_name"' ('"$parent_short"').  Run

  git reset '"$parent_name"' -- "'"$file"'"
  git submodule update -- "'"$file"'"

to checkout the newer version of the submodule in your work tree.
Then try the commit again.

If you know what you are doing, run
  git config hooks."'"$file"'".update "'"$override"'"
to override this hook.
'
  return 1
}

#----------------------------------------------------------------------------

# Check diffs to first parent of commit under construction.
readonly diffs="$(index_diffs HEAD)"
readonly diffs_module="$(echo "$diffs" | filter_diffs_module)"
bad="$(
test -n "$diffs_module" && echo "$diffs_module" |
while read src_mode dst_mode src_obj dst_obj status file; do
  check_module_rewind HEAD ||
  break
done
)"
test -z "$bad" || die "$bad"

# Check diffs to second parent of commit under construction during a
# merge ("git commit" after "git merge" with conflicts or --no-commit).
readonly merge_head="$(git rev-parse -q --verify MERGE_HEAD)"
if test -n "$merge_head"; then
  readonly merge_diffs="$(index_diffs MERGE_HEAD)"
else
  readonly merge_diffs=''
fi
readonly merge_diffs_module="$(echo "$merge_diffs" | filter_diffs_module)"
bad="$(
test -n "$merge_diffs_module" && echo "$merge_diffs_module" |
while read src_mode dst_mode src_obj dst_obj status file; do
  check_module_rewind MERGE_HEAD ||
  break
done
)"
test -z "$bad" || die "$bad"
