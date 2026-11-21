#!/bin/bash
# FULL regression with the final code (keepalive fix + final ASCENT trigger): 7-cell sharp/curved matrix, Floor-1-only round trips (loaded, unloaded; existing arc-turn driver),
# Floor-1+2 round trips (full MPPI-driven two-ramp trip loaded + unloaded, and the raw-cmd_vel ramp-2 traversal unloaded). Guarded clean-host protocol throughout.
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; P8=$WS/validation/phase8; D=$P8/fsm_multiramp; O=$P8/halt_loop/run9_regression; CS=$P8/construction_site_descent
export HG_LOW_RTF=0.85
echo "### matrix"; LOGDIR=$O/matrix bash $P8/run_matrix_regression_cleanhost.sh > $O/matrix_full.log 2>&1; echo "matrix exit=$?"
echo "### floor-1 round trips"
$D/guarded_run.sh $O rt_loaded   100.0 0.1 bash -c "cd $CS && python3 arc_turn_followpath_test_ekf.py $O/rt_loaded.csv"
$D/guarded_run.sh $O rt_unloaded 0.0   0.0 bash -c "cd $CS && python3 arc_turn_followpath_test_ekf.py $O/rt_unloaded.csv"
echo "### floor-1+2 full MPPI round trips"
$D/guarded_run.sh $O mr_unloaded 0.0   0.0 python3 $D/multiramp_nav2_test.py $O/mr_unloaded.json --legs all
$D/guarded_run.sh $O mr_loaded   100.0 0.1 python3 $D/multiramp_nav2_test.py $O/mr_loaded.json --legs all
echo "### floor-2 ramp-2 traversal (raw cmd_vel, ground-truth gated)"
$D/guarded_run.sh $O ramp2_trav  0.0   0.0 bash -c "cd $P8/floor2 && python3 ramp2_traversal_test.py $O/ramp2_trav.csv"
echo REGRESSION DONE
