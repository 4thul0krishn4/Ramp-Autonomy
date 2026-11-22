#!/bin/bash
export HG_LOW_RTF=0.85
D=${RAMP_WS:?}/validation/phase8/halt_loop
$D/guarded_run.sh $D/run1_baseline base_unloaded 0.0 0.0 python3 $D/start_stop_repro.py $D/run1_baseline/base_unloaded.json 24
$D/guarded_run.sh $D/run1_baseline base_loaded 100.0 0.1 python3 $D/start_stop_repro.py $D/run1_baseline/base_loaded.json 24
echo CHAIN1 DONE
