#!/bin/bash
# Syncs the working copy of NairdaArduinoLibrary to the Arduino libraries folder
# Excludes .git and other non-library files

SRC="/Users/adrianalvarezgalicia/flutter projects/nairda_workspace/NairdaArduinoLibrary/"
DEST="/Users/adrianalvarezgalicia/Documents/Arduino/libraries/NairdaArduinoLibrary/"

rsync -a --delete \
  --exclude='.git/' \
  --exclude='.gitignore' \
  --exclude='.DS_Store' \
  --exclude='sync_to_arduino.sh' \
  "$SRC" "$DEST"

echo "Synced NairdaArduinoLibrary to Arduino libraries."
