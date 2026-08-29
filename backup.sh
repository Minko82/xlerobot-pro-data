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
FAILED=0

# Discover the directories on the robot rather than listing them here. A
# hardcoded list silently skips whatever it does not name -- A1 was missed
# exactly that way, and the omission was invisible because the script still
# reported success.
DIRS=$(ssh -o BatchMode=yes "xle@$HOST" "cd $SRC 2>/dev/null && ls -d */ 2>/dev/null | tr -d /" \
       | grep -vx scripts)
echo "  syncing: $(echo $DIRS | tr '\n' ' ')"
for d in $DIRS; do
    printf '  %-12s ' "$d"
    mkdir -p "$DEST/$d"
    if rsync -a "xle@$HOST:$SRC/$d/" "$DEST/$d/"; then
        echo "ok  ($(find "$DEST/$d" -type f | wc -l | tr -d ' ') files)"
    else
        echo "FAILED"; FAILED=1
    fi
done

printf '  %-12s ' "vla_logs"
rsync -a "xle@$HOST:vla/*.log" "$DEST/vla_logs/" 2>/dev/null && echo "ok" || echo "none"

after=$(du -sm "$DEST" | cut -f1)
[ "$FAILED" = 1 ] && echo "WARNING: at least one directory failed to sync" >&2
echo "backed up from $HOST -- $DEST now $(du -sh "$DEST" | cut -f1) (was ${before} MB, now ${after} MB)"
