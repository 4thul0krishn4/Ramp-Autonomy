#!/bin/bash
D=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}/validation/phase8/delivery; O=$D/run5_rtf_states
export HG_LOW_RTF=0.85 SPAWN_X=79.5 SPAWN_Y=0.0 SPAWN_Z=6.68
hostok() { [ "$(cat /sys/class/power_supply/ACAD/online)" = 1 ] && [ "$(powerprofilesctl get)" = performance ] && [ "$(cat /sys/devices/system/cpu/cpu0/cpufreq/energy_performance_preference)" = performance ]; }
for rep in 7 8 9; do
  for st in detached_idle detached_away; do
    powerprofilesctl set performance 2>/dev/null
    w=0; until hostok; do sleep 20; w=$((w+20)); [ $w -ge 900 ] && { echo "HOST NOT ON AC/performance for 15 min - aborting"; exit 4; }; done
    echo "### rep $rep state $st: $(date +%T) ac=$(cat /sys/class/power_supply/ACAD/online) profile=$(powerprofilesctl get) epp=$(cat /sys/devices/system/cpu/cpu0/cpufreq/energy_performance_preference) avgMHz=$(grep MHz /proc/cpuinfo | awk '{s+=$4;n++} END{print int(s/n)}')"
    DETACHABLE=true $D/guarded_run.sh $O ${st}_r$rep 100.0 0.1 python3 $D/rtf_state_run.py $st $O/${st}_r$rep.json
  done
done
echo TOPUP DONE
