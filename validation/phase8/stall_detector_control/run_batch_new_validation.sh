#!/bin/bash
# Validation of the FIXED detector: 6 normal sessions (false-positive rate) interleaved with 6 freeze
# sessions (true-positive rate on the positive control), alternating loaded/unloaded.
H=${RAMP_WS:?}/validation/phase8/stall_detector_control
for spec in "normal loaded 31" "freeze loaded 41" "normal unloaded 32" "freeze unloaded 42" \
            "normal loaded 33" "freeze loaded 43" "normal unloaded 34" "freeze unloaded 44" \
            "normal loaded 35" "freeze loaded 45" "normal unloaded 36" "freeze unloaded 46"; do
  set -- $spec
  bash $H/run_session.sh new $1 $2 $3 || { echo "BATCH STOPPED at: $spec"; exit 2; }
done
echo "BATCH new-validation COMPLETE"
