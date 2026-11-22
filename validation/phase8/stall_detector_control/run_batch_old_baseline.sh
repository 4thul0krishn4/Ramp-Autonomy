#!/bin/bash
# Baseline with the UNMODIFIED (installed) stall detector: 3 freeze sessions + 2 normal sessions.
H=${RAMP_WS:?}/validation/phase8/stall_detector_control
for spec in "freeze loaded 11" "freeze unloaded 12" "freeze loaded 13" "normal loaded 21" "normal unloaded 22"; do
  set -- $spec
  bash $H/run_session.sh old $1 $2 $3 || { echo "BATCH STOPPED at: $spec"; exit 2; }
done
echo "BATCH old-baseline COMPLETE"
