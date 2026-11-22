#!/bin/bash
# Repeated real-time RTF runs per payload state (see rtf_state_run.py). Host pre-check before EVERY run (AC online, power profile performance, EPP performance); a run whose host_during shows
# otherwise is flagged INVALID by the analysis. 6 reps, state order rotated each rep.
D=${RAMP_WS:?}/validation/phase8/delivery; O=$D/run5_rtf_states
export HG_LOW_RTF=0.85 SPAWN_X=79.5 SPAWN_Y=0.0 SPAWN_Z=6.68
STATES=(attached attached_plugin detached_idle detached_away unloaded)
hostok() { [ "$(cat /sys/class/power_supply/ACAD/online)" = 1 ] && [ "$(powerprofilesctl get)" = performance ] && [ "$(cat /sys/devices/system/cpu/cpu0/cpufreq/energy_performance_preference)" = performance ]; }
for rep in 1 2 3 4 5 6; do
  for k in 0 1 2 3 4; do
    st=${STATES[$(( (k + rep - 1) % 5 ))]}
    powerprofilesctl set performance 2>/dev/null
    w=0; until hostok; do sleep 20; w=$((w+20)); [ $w -ge 900 ] && { echo "HOST NOT ON AC/performance for 15 min - aborting chain"; exit 4; }; done
    case $st in attached) DET=false; M=100.0; C=0.1;; attached_plugin|detached_idle|detached_away) DET=true; M=100.0; C=0.1;; unloaded) DET=false; M=0.0; C=0.0;; esac
    echo "### rep $rep state $st: $(date +%T) ac=$(cat /sys/class/power_supply/ACAD/online) profile=$(powerprofilesctl get) epp=$(cat /sys/devices/system/cpu/cpu0/cpufreq/energy_performance_preference) avgMHz=$(grep MHz /proc/cpuinfo | awk '{s+=$4;n++} END{print int(s/n)}')"
    DETACHABLE=$DET $D/guarded_run.sh $O ${st}_r$rep $M $C python3 $D/rtf_state_run.py $st $O/${st}_r$rep.json
  done
done
echo RTF STATES DONE
