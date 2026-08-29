#!/bin/bash
# Full backup of every artefact the Jetson produces. Run after each session.
# The Jetson's SD card has accumulated ext4 errors from power loss, so this is
# the only durable copy. Note macOS ships rsync 2.6.9 -- no --info= flag.
set -e
DEST=~/Desktop/Workspace/xlerobot-pro-data
mkdir -p "$DEST/vla_logs" "$DEST/calibration"
rsync -a xle@10.0.0.197:xlerobot-pro/results/ "$DEST/"                    # A2 + thermal
rsync -a "xle@10.0.0.197:vla/*.log" "$DEST/vla_logs/" 2>/dev/null || true # benchmark consoles
rsync -a xle@10.0.0.197:xlerobot-pro/calibration/ "$DEST/calibration/"    # reference pose
echo "backed up $(du -sh "$DEST" | cut -f1) to $DEST"
