#!/bin/bash
# Re-bind the HC-05 RFCOMM link used for robot telemetry (rat_sw project).
# Does nothing else: fixed device MAC, fixed channel, fixed rfcomm slot.
set -e
rfcomm release 0 >/dev/null 2>&1 || true
rfcomm bind 0 20:13:09:11:04:32 1
