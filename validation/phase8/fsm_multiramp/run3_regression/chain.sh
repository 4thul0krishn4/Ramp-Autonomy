#!/bin/bash
export HG_LOW_RTF=0.85
cd ${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}/validation/phase8/fsm_multiramp
./guarded_run.sh run3_regression rt_loaded 100.0 0.1 bash -c "cd ${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}/validation/phase8/construction_site_descent && python3 arc_turn_followpath_test_ekf.py ${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}/validation/phase8/fsm_multiramp/run3_regression/rt_loaded.csv"
./guarded_run.sh run3_regression rt_unloaded 0.0 0.0 bash -c "cd ${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}/validation/phase8/construction_site_descent && python3 arc_turn_followpath_test_ekf.py ${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}/validation/phase8/fsm_multiramp/run3_regression/rt_unloaded.csv"
echo REGRESSION CHAIN DONE
