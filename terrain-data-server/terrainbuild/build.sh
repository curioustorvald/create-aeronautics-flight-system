#!/usr/bin/env bash
# Builds terrainbuild.jar from src/. Needs a JDK 17 or newer (records, text blocks).
# Put the resulting jar beside your server launcher -- its default --world is the
# "world" folder next to the jar -- along with terrain-update.sh if you want it on cron.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
javac --release 17 -d "$out" $(find src -name '*.java')
jar cfm terrainbuild.jar MANIFEST.MF -C "$out" .
echo "built $(pwd)/terrainbuild.jar"
