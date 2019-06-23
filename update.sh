#!/bin/bash

# run from parent directory.
# usage:
#   include/update.sh

src="../include"
dest="include"

for f in "$src"/*; do
  fname="${f#"$src"/}"
  f2="$dest/$fname"
  if [ -f "$f" -a -f "$f2" -a "$f" -nt "$f2" ]; then
    if [ -L "$f" ]; then
      echo "can't handle symlink \"$f\" yet." >&2
      continue;
    fi
    if [ -L "$f2" ]; then
      rm "$f2"
    fi
    echo "installing new version of \"$fname\"."
    echo cp -a "$f" "$f2"
  fi
done
