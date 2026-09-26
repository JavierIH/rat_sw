#!/bin/bash
# tools/robot.sh [-w "pattern"] [-t seconds] "CMD1" "CMD2" ...: sends the commands
# through tools/bt_logger.py's FIFO (never blocks: fails if no logger is reading),
# waits until a received line matches the pattern (default: nothing, 3 s),
# and prints the new human-readable lines.
S=${RAT_BT_DIR:-/tmp/rat_bt}
WAIT=""; TIMEOUT=3
while getopts "w:t:" o; do case $o in w) WAIT=$OPTARG;; t) TIMEOUT=$OPTARG;; esac; done
shift $((OPTIND-1))
pgrep -f "bt_logger.py" >/dev/null || { echo "!! el registrador no esta en marcha"; exit 1; }
L=$(cat $S/bt_log_path)
N=$(wc -l < "$L")
for c in "$@"; do
    timeout 2 bash -c "echo '$c' > $S/bt_in" || { echo "!! no se pudo enviar: $c"; exit 1; }
    sleep 1.2
done
if [ -n "$WAIT" ]; then
    for i in $(seq "$TIMEOUT"); do
        tail -n +$((N+1)) "$L" | grep -v "	>	" | grep -qE "$WAIT" && break
        sleep 1
    done
    sleep 1
else
    sleep "$TIMEOUT"
fi
tail -n +$((N+1)) "$L" | grep -v "	<	@" | cut -f1,3
