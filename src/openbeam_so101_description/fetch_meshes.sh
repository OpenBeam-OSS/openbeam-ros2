#!/bin/sh
# Fetch the SO-101 meshes the URDF uses from TheRobotStudio/SO-ARM100 at a pinned
# commit and verify each against meshes/SHA256SUMS. Apache-2.0; see NOTICE.
set -eu
COMMIT=a758567c3978dfeefe282ede0500085a48fe8f78
BASE="https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/${COMMIT}/Simulation/SO101/assets"
cd "$(dirname "$0")/meshes"
while read -r sum file; do
  if [ -f "$file" ] && echo "$sum  $file" | sha256sum -c - >/dev/null 2>&1; then continue; fi
  curl -fsSL --retry 3 -o "$file.part" "$BASE/$file"
  echo "$sum  $file.part" | sha256sum -c - >/dev/null || { echo "checksum mismatch: $file" >&2; rm -f "$file.part"; exit 1; }
  mv "$file.part" "$file"
done < SHA256SUMS
echo "SO-101 meshes OK (SO-ARM100 ${COMMIT})"
