# Gazebo GUI crash - real cause, fix, and how to watch the simulation live (2026-09-20)

## Result: the GUI works. Launch it with `visualization/run_live_gui.sh loaded` (or `unloaded`, or add `--no-trip` to just bring the world up).

## The real error (captured, not assumed) - `gui_diagnosis/attempt0_stderr.log`
Running `gz sim -v 4 -r <world>` with the GUI enabled from the VS Code terminal died immediately with:

    gz sim gui: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE

## What it actually was (proven with `LD_DEBUG=libs`, `gui_diagnosis/lddebug_stderr.log`)
* Gazebo here is NOT a snap: it is the apt/ROS Jazzy build (gz-sim 8.11 vendor + apt libgz-*). "A snap library conflict" was half right: the conflict comes from the terminal you launch it from.
  The VS Code snap exports `GTK_PATH=/snap/code/264/usr/lib/x86_64-linux-gnu/gtk-3.0` and `GTK_EXE_PREFIX=/snap/code/264/usr` into every terminal/child process (`gui_diagnosis/env_vscode_terminal.txt`).
* The Gazebo GUI (Qt/GTK) initialises GTK, which loads its modules from `GTK_PATH` first. It picks `/snap/code/264/usr/lib/x86_64-linux-gnu/gtk-3.0/modules/libcanberra-gtk-module.so`, whose RPATH is `/snap/core20/current/lib/x86_64-linux-gnu`.
  That drags core20's old glibc `libpthread.so.0` into a process running the host's newer glibc, and the dynamic loader aborts with the symbol error above. `LD_LIBRARY_PATH` was clean (no snap entries), so it was not an OGRE/Qt library conflict.
* The ROS launch file is not sanitising anything: the server runs fine; only the GUI client process dies, because it inherits the poisoned GTK variables.

## Fix and the attempts (in the order you asked, each with a real launch)
1. **Unset `GTK_PATH` and `GTK_EXE_PREFIX` before launch - WORKED on the first try.** `env -u GTK_PATH -u GTK_EXE_PREFIX gz sim -v 3 -r <world>`: `gz sim gui` stayed alive, a "Gazebo Sim" window (1000x845) appeared, and a window capture shows the ramp world rendered
   with the entity tree and real-time factor 99.89% (`gui_diagnosis/attempt1_gui_window.png`). Only harmless QML "binding loop" warnings on stderr (`attempt1_stderr.log`).
2. Checking for a conflicting non-snap OGRE/Qt on `LD_LIBRARY_PATH`: not needed (it has no snap or foreign OGRE/Qt entries; `LD_DEBUG` located the real culprit).
3. Running the snap's own binary instead of via ROS launch: not applicable (Gazebo is not a snap).
Then the real thing: the complete stack (`ramp_autonomy_bringup`, `world:=construction_site`, loaded, `headless:=false`) launched with the same two variables unset, GUI window up, camera set to follow `ramp_robot` (`gz service /gui/follow`), and the full round trip run against it.

## One harness change (tooling, not the autonomy stack)
`validation/phase8/harness/harness_guard.py` counted the GUI's three processes (`gz sim` launcher, `gz sim server`, `gz sim gui`) as three simulator instances and refused to run ("need exactly 1"). It now counts INSTANCES: the GUI client is not an instance and a launcher that has a `gz sim server` child is not one either.
Unit-tested on synthetic process tables (headless = 1, GUI = 1, two headless = 2, two GUI = 2, GUI + leaked headless = 2, GUI + orphan server = 2) and on the live GUI stack (verify OK, RTF 0.992). Backup: `harness_guard.BEFORE_GUI_AWARE.py`. No validated autonomy code was touched.

## The live run that was watched (loaded, construction_site, full current stack, GUI on) - `../validation/phase8/construction_site_descent/roundtrip_live_gui_loaded.{log,csv}`
All four legs ok, no abort, ascent physically verified (ground-truth vs EKF disagreement +0.019 / +0.037 / -0.019 m on legs 1 / 2 / 4), leg-3 goal verification 101%, 18 HALT events (5 in the descent), min stability margin 0.463, final leg-4 position (2.02, +1.24), min RTF 0.991 with the GUI rendering.

## Notes
* The variables are only a problem when launching from the VS Code snap terminal. From a normal terminal (GNOME Terminal etc.) plain `ros2 launch ... headless:=false` should work; the wrapper unsets them either way (harmless).
* The GUI adds a little load (RTF 0.991-0.994 vs 0.997+ headless). The validation harness stays headless.
