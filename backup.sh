#!/bin/bash
# Full backup of every artefact the Jetson produces. Run after each session.
# The Jetson's SD card has accumulated ext4 errors from power loss, so this is
# the only durable copy. Note macOS ships rsync 2.6.9 -- no --info= flag.
#
# Additive only: no --delete anywhere. A file removed on the robot stays here.
# That is deliberate -- this is a backup, not a mirror.
#
# scripts/ is NOT synced back from the robot. It is version controlled here and
# pushed outwards; pulling it back would overwrite the authoritative copy with
# whatever happened to be on the SD card.
set -u
HOST="${1:-xle-desktop.local}"
DEST=~/Desktop/Workspace/xlerobot-pro-data
SRC=xlerobot-pro-data          # where the run data actually lives on the robot

if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "xle@$HOST" true 2>/dev/null; then
    echo "cannot reach xle@$HOST" >&2
    echo "try:  $0 192.168.1.14" >&2
    exit 1
fi

mkdir -p "$DEST/vla_logs" "$DEST/calibration"
before=$(du -sm "$DEST" | cut -f1)

for d in A2 B1 thermal calibration protocols; do
    printf '  %-12s ' "$d"
    if ssh -o BatchMode=yes "xle@$HOST" "test -d $SRC/$d" 2>/dev/null; then
        rsync -a "xle@$HOST:$SRC/$d/" "$DEST/$d/" && echo "ok"
    else
        echo "absent on robot, skipped"
    fi
done

printf '  %-12s ' "vla_logs"
rsync -a "xle@$HOST:vla/*.log" "$DEST/vla_logs/" 2>/dev/null && echo "ok" || echo "none"

after=$(du -sm "$DEST" | cut -f1)
echo "backed up from $HOST -- $DEST now $(du -sh "$DEST" | cut -f1) (was ${before} MB, now ${after} MB)"
