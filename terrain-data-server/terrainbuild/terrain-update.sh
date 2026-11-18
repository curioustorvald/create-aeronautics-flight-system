#!/usr/bin/env bash
# Incremental re-survey of the GPWS terrain database.
#
# Safe to run against a LIVE server. It only ever READS world/region and only ever WRITES
# the CC disk, it is niced and I/O-idle, and it treats a chunk caught mid-flush as "come
# back next pass" rather than as an error -- the region's mtime will bring it round again.
#
# By default it does the ground under the fleet: computer #68 writes activity.txt with
# every aircraft it has heard from, and the builder restricts itself to regions within
# --radius of one of them. With no aircraft about it falls back to the most
# recently-modified changed regions, which is the same instinct -- that is where the
# players have been.
#
# Run it from cron every few minutes:
#     */5 * * * * /path/to/your/server/terrain-update.sh
set -euo pipefail

SERVER="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCK="$SERVER/terrainbuild.lock"
LOG="$SERVER/logs/terrainbuild.log"
mkdir -p "$(dirname "$LOG")"

# One at a time. A pass that overruns its interval must not stack up behind itself.
exec 9>"$LOCK"
if ! flock -n 9; then
    echo "$(date -Is) update still running, skipping this tick" >> "$LOG"
    exit 0
fi

{
    echo "--- $(date -Is) ---"
    nice -n 15 ionice -c 3 java -Xmx2g -jar "$SERVER/terrainbuild.jar" update \
        --threads 2 --max-regions 64 --throttle-ms 25 --radius 8000 "$@"
} >> "$LOG" 2>&1

# Keep the log from growing without bound.
if [ "$(stat -c%s "$LOG")" -gt 2000000 ]; then
    tail -c 500000 "$LOG" > "$LOG.tmp" && mv "$LOG.tmp" "$LOG"
fi
