#!/bin/bash
# Other cells that showed HALT thrash historically: sharp/normal/loaded (FAIL 8-10 HALTs in matrices), sharp/0.45/loaded (24-HALT original run, 10 in the last matrix), curved/0.45/unloaded (HALT_THRASH 8-11 in earlier matrices), curved/normal/loaded.
# Interleaved PRE vs FIX, 6+6 each.
export HG_LOW_RTF=0.85 AB_FROM=1 AB_TO=6
D=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}/validation/phase8/halt_loop
run() { mkdir -p $D/$4; WORLD=$1 MASS=$2 CG=$3 GZW=$5 PREFIX=$6 OUTDIR=$D/$4 $D/persist_ab.sh > $D/$4/driver.log 2>&1; }
run sharp 100.0 0.1 run6_sharp_normal_loaded_ab ramp_world_sharp sharp_loaded_normal
run sharp_lowfriction_TESTONLY 100.0 0.1 run6_sharp_mu045_loaded_ab ramp_world_sharp sharp_loaded_mu045
run curved_mu045_TESTONLY 0.0 0.0 run6_curved_mu045_unloaded_ab ramp_world_curved curved_unloaded_mu045
run curved 100.0 0.1 run6_curved_normal_loaded_ab ramp_world_curved curved_loaded_normal
echo CHAIN5 DONE
