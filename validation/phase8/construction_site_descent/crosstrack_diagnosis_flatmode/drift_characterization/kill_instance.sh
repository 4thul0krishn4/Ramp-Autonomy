#!/bin/bash
# Kill every process belonging to ONE isolated sim instance: any process whose
# environment carries GZ_PARTITION=<partition>. Usage: kill_instance.sh <partition>
# Needed for parallel trials - a global pkill-by-name would kill the other
# instances. (Name-based pkill was also found to miss child nodes, which is
# what leaked 57 processes and starved the CPU in the earlier sequential runs.)
part="$1"
[ -z "$part" ] && { echo "usage: $0 <partition>"; exit 2; }
for d in /proc/[0-9]*; do
  pid=${d#/proc/}
  [ "$pid" = "$$" ] && continue
  if tr '\0' '\n' < "$d/environ" 2>/dev/null | grep -qx "GZ_PARTITION=$part"; then
    kill -9 "$pid" 2>/dev/null
  fi
done
exit 0
