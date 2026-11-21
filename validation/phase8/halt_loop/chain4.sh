#!/bin/bash
# T1: loop propensity on the start-stop reproduction with an on-demand optimizer reset per leg (INJECT=1), loaded 100 kg, 24 legs, three arms:
#   base  = final code before this session (state machine 4db40158 + switcher 3020997a)
#   B     = base + switcher that does not push a profile on HALT / HALT recovery (candidate fix 2)
#   fix   = persistence-gated slip HALT (candidate fix 1), everything else as base
export HG_LOW_RTF=0.85 INJECT=1
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; D=$WS/validation/phase8/halt_loop; I=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav; S=$WS/src/ramp_autonomy_nav/ramp_autonomy_nav
mkdir -p $D/run4_arms
run() { $D/guarded_run.sh $D/run4_arms $1 100.0 0.1 python3 $D/start_stop_repro.py $D/run4_arms/$1.json 24; }
for rep in 1 2; do
  cp $D/state_machine_node.PRE_PERSIST.py $I/state_machine_node.py; cp $S/nav2_param_switcher_node.py $I/nav2_param_switcher_node.py; echo "### base_r$rep fsm $(sha256sum $I/state_machine_node.py | cut -c1-8)"; run base_r$rep
  cp $D/nav2_param_switcher_node.EXP_NOHALTPUSH.py $I/nav2_param_switcher_node.py; echo "### B_r$rep switcher $(sha256sum $I/nav2_param_switcher_node.py | cut -c1-8)"; run B_r$rep
  cp $S/nav2_param_switcher_node.py $I/nav2_param_switcher_node.py; cp $S/state_machine_node.py $I/state_machine_node.py; echo "### fix_r$rep fsm $(sha256sum $I/state_machine_node.py | cut -c1-8)"; run fix_r$rep
done
cp $D/state_machine_node.PRE_PERSIST.py $I/state_machine_node.py; cp $S/nav2_param_switcher_node.py $I/nav2_param_switcher_node.py
echo "left with pre-persist FSM installed for now: $(sha256sum $I/state_machine_node.py | cut -c1-8)"; echo CHAIN4 DONE
