#!/usr/bin/env bash
# Stable build wrapper so the command string never changes (one approval covers all builds).
set -e
export IDF_PATH=/Users/robertfowler/.espressif/v6.0.3/esp-idf
. "$IDF_PATH/export.sh" >/dev/null 2>&1
cd /Users/robertfowler/walocal/ESC
idf.py build 2>&1 | tee /tmp/esc_build.log | tail -n 60
