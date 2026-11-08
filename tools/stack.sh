#!/usr/bin/env bash
# stack.sh (v10.4) - start / stop the whole simulation stack with one command.
#
#   stack.sh stop                  stop everything (sim, Nav2, RViz, probe, runner, guard);
#                                  anything still running after 10 s is killed
#   stack.sh start                 T1-T5 as tabs: Gazebo, localizer, map, Nav2, RViz,
#                                  plus a 'lanes' tab that draws the keep-left lanes in RViz
#   stack.sh start <label>         the same, plus T6: the probe recording under <label>
#   stack.sh start <label> guard camera
#                                  the same with the camera: the robot gets its camera (your
#                                  URDF is checked unchanged first), eye.py <label> finds people
#                                  in it with the detector on the Mac, and an 'eye view' window
#                                  shows what it sees. Saved: ~/runs/eye/<label>.csv, ~/runs/eye_last.log
#   stack.sh start <label> guard camera scene [nocam]
#                                  v8.2: also put a standing picker and a look-alike post into
#                                  aisle 2 (scene.py; ~/runs/scene.json) for picker_test.py; no probe
#                                  tab (it measures the aisle-1 worker). nocam: the guard ignores
#                                  the camera (GUARD_CAMERA=0, lidar only like v7.1), for a baseline.
#                                  Any start with a label also records CPU and memory every 2 s
#                                  (diag8.py watch -> ~/runs/load/<label>.csv).
#   stack.sh start <label> guard camera traffic headless [novehicles]
#                                  v10: the tour. traffic: traffic.py spawns two pickers, a person
#                                  who steps out, a visitor, a tugger and a forklift, and a
#                                  'traffic' tab drives the vehicles (~/runs/traffic/<label>.csv);
#                                  then tour_test.py <label> drives laps through both aisles.
#                                  headless: Gazebo without its window (the window alone used ~2
#                                  CPU cores; RViz shows everything). novehicles: the guard
#                                  ignores vehicles (GUARD_VEHICLES=0, people only like v9), for a
#                                  baseline. No probe tab with traffic.
#   stack.sh start <label> guard camera traffic headless clean
#                                  v10.3: a cleaning shift instead of the tour: a 'clean' tab runs your
#                                  coverage_nav2 (warehouse_planner) waiting for a command
#                                  (auto_start:=false), with its rescue reverse sent through the
#                                  gate and the Collision Monitor like Nav2's own commands
#                                  (/cmd_vel -> /cmd_vel_raw; on its own it drives the wheels
#                                  directly, past the guard), and a 'coverage rec' tab runs your
#                                  coverage_recorder. Then start the clean with a /mission_command
#                                  (or llm_commander); tour_test.py <label> watch measures it.
#                                  Saved: ~/runs/logs/clean_<label>.log (and clean_last.log)
#                                  v10.4: coverage_nav2 runs on simulation time (use_sim_time:=true:
#                                  on the wall clock its 15 s stall timer was 7 s of robot time at
#                                  0.48 x real time) and reads the guard's /people_guard/state.
#                                  v10.4: and a 'robot says' tab (narrator.py: the robot says what
#                                  it is doing, as it happens; ~/runs/logs/say_<label>.log).
#   stack.sh start <label> guard camera traffic headless clean talk
#                                  v10.4: also a 'talk' tab: llm_commander, to give orders and ask
#                                  questions in plain words ("clean aisle 2", "why did you stop?"),
#                                  with Ollama on the Mac (OLLAMA_HOST=0.0.0.0 ollama serve; llama3.2).
#   stack.sh start <label> guard   the same, but Nav2 comes up behind the fail-safe gate and
#                                  the Collision Monitor, reading the guard's /scan_nav
#                                  (nav2_people.launch.py), the wiring is checked, Nav2 is
#                                  given 25 s (not 10) without progress before it gives up
#                                  on a goal (waiting for people is not being stuck), and a
#                                  'guard' tab runs people_guard.py <label>. Saved for the report:
#                                    ~/runs/guard_last.log      its GO / WAIT lines
#                                    ~/runs/guard/<label>.csv   every decision
#                                    ~/runs/nav2_last.log       the Nav2 tab's output
#
# start opens each part only when the one before is ready, and stops with a
# message if something doesn't come up. Tabs stay open after their program
# exits, so you can still read the log. RViz runs on simulation time.
# no "set -u": the ROS setup scripts read unset variables
WS="$HOME/ros2_ws"
TOOLS="$WS/src/tools"
# Only processes this stack starts. An editor with nav2_params.yaml or the
# probe open is safe: ROS nodes match by install path, our scripts only when
# python3 is running them.
P='ign gazebo|ros2 launch|ros2 run|/opt/ros/humble/lib/|lib/warehouse_planner/|python3 [^ ]*(worker_clearance|people_trials|lanes|people_guard|guard_gate|check_wiring|eye|scene|picker_test|diag8|traffic|tour_test|narrator)\.py'
# the detector on the Mac: the VM's default gateway is the Mac (UTM shared network 192.168.64.1,
# QEMU emulated network 10.0.2.2)
MAC_IP="$(ip route 2>/dev/null | awk '/^default/ {print $3; exit}')"
EYE_SERVER="${EYE_SERVER:-http://${MAC_IP:-10.0.2.2}:8765}"

tab() {  # title, command
  gnome-terminal --tab -- bash -ic "printf '\\033]0;$1\\007'; cd $WS && source install/setup.bash && $2; exec bash"
}

wait_for() {  # what, check command, timeout seconds
  local t=0
  printf '  waiting for %s ' "$1"
  until bash -c "$2" >/dev/null 2>&1; do
    sleep 2
    t=$((t + 2))
    printf '.'
    if [ "$t" -ge "$3" ]; then
      echo " NOT READY after ${3} s. Look at that tab, then run: stack.sh stop"
      return 1
    fi
  done
  echo ' ok'
}

case "${1:-}" in
  stop)
    pkill -f "$P"
    for i in 1 2 3 4 5; do
      sleep 2
      pgrep -f "$P" >/dev/null || break
    done
    if pgrep -f "$P" >/dev/null; then
      echo 'These did not stop within 10 s, forcing them:'
      pgrep -af "$P"
      pkill -9 -f "$P"
      sleep 2
    fi
    if pgrep -af "$P"; then
      echo "Still running (listed above). Run 'stack.sh stop' again."
    else
      echo 'All clear.'
    fi
    ;;
  start)
    label=''; mode=''; camera=''; scene=''; gcam=1; traffic=''; headless=0; gveh=1; clean=''; talk=''
    for a in "${@:2}"; do
      case "$a" in
        guard) mode=guard ;;
        camera) camera=yes ;;
        scene) scene=yes ;;
        nocam) gcam=0 ;;
        traffic) traffic=yes ;;
        headless) headless=1 ;;
        novehicles) gveh=0 ;;
        clean) clean=yes ;;
        talk) talk=yes ;;
        *) if [ -z "$label" ]; then label="$a"; else
             echo "Unknown option '$a'. usage: stack.sh start [label] [guard] [camera] [scene] [nocam] [traffic] [headless] [novehicles] [clean] [talk]"; exit 1; fi ;;
      esac
    done
    if pgrep -f "$P" >/dev/null; then
      echo "Something from the last run is still going. Run 'stack.sh stop' first."
      exit 1
    fi
    if [ -n "$clean" ] && [ "$mode" != guard ]; then
      echo "clean needs guard (the cleaning runs behind the guard): stack.sh start <label> guard camera traffic headless clean"
      exit 1
    fi
    if [ -n "$talk" ] && [ -z "$clean" ]; then
      echo "talk needs clean (it talks to the cleaning): stack.sh start <label> guard camera traffic headless clean talk"
      exit 1
    fi
    source "$WS/install/setup.bash"
    mkdir -p "$HOME/runs/logs"
    # v8.2: with a label, each tab's output is also kept as ~/runs/logs/<tab>_<label>.log
    # (the *_last.log files are overwritten by the next start)
    keep_nav2=''; keep_guard=''; keep_eye=''; keep_traffic=''; keep_clean=''
    if [ -n "$label" ]; then
      keep_nav2="$HOME/runs/logs/nav2_$label.log"; keep_guard="$HOME/runs/logs/guard_$label.log"
      keep_eye="$HOME/runs/logs/eye_$label.log"; keep_traffic="$HOME/runs/logs/traffic_$label.log"
      keep_clean="$HOME/runs/logs/clean_$label.log"
    fi
    if [ -n "$camera" ]; then
      python3 "$TOOLS/check_camera_urdf.py" || { echo 'Stopped: the camera URDF check failed (nothing was started).'; exit 1; }
      # settings go into the tab's command line (new tabs don't inherit this shell's variables)
      tab 'T1 gazebo+camera' "SIM_HEADLESS=$headless EYE_WIDTH=${EYE_WIDTH:-640} EYE_HEIGHT=${EYE_HEIGHT:-480} EYE_RATE=${EYE_RATE:-3} ros2 launch $TOOLS/launch_sim_camera.launch.py"
    elif [ "$headless" = 1 ]; then
      tab 'T1 gazebo (no window)' "SIM_CAMERA=0 SIM_HEADLESS=1 ros2 launch $TOOLS/launch_sim_camera.launch.py"
    else
      tab 'T1 gazebo' 'ros2 launch warehouse_cleaning_robot launch_sim.launch.py'
    fi
    wait_for 'Gazebo' 'timeout 5 ros2 topic echo /clock --once' 120 || exit 1
    if [ -n "$label" ]; then
      nohup python3 "$TOOLS/diag8.py" watch "$label" >/dev/null 2>&1 &
      echo "  recording CPU and memory every 2 s -> ~/runs/load/$label.csv"
    fi
    rm -f "$HOME/runs/scene.json"          # eye.py scores only a scene spawned in this run
    if [ -n "$scene" ]; then
      python3 "$TOOLS/scene.py" spawn || echo '  note: the scene is not complete (see the line above); picker_test.py measures what is there'
    fi
    rm -f "$HOME/runs/traffic/spawned.json"
    if [ -n "$traffic" ]; then
      python3 "$TOOLS/traffic.py" spawn || echo '  note: not everyone was spawned (see the line above); tour_test.py measures who is there'
    fi
    if [ -n "$camera" ]; then
      wait_for 'the camera' 'timeout 8 ros2 topic echo /camera/image_raw --once --field header' 90 || exit 1
    fi
    tab 'T2 localizer' 'ros2 run warehouse_planner ground_truth_localizer'
    sleep 5
    tab 'T3 map' 'ros2 launch warehouse_cleaning_robot show_map.launch.py'
    wait_for 'the map' 'timeout 5 ros2 topic echo /map --once --field info --qos-durability transient_local --qos-reliability reliable' 60 || exit 1
    if [ "$mode" = guard ]; then
      tab 'T4 nav2+monitor' "PYTHONUNBUFFERED=1 ros2 launch $TOOLS/nav2_people.launch.py 2>&1 | tee $HOME/runs/nav2_last.log $keep_nav2"
    else
      tab 'T4 nav2' 'ros2 launch warehouse_cleaning_robot nav2_slam.launch.py'
    fi
    wait_for 'Nav2' "timeout 5 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger | grep -q 'success=True'" 120 || exit 1
    if [ "$mode" = guard ]; then
      wait_for 'the Collision Monitor' "timeout 5 ros2 service call /lifecycle_manager_people/is_active std_srvs/srv/Trigger | grep -q 'success=True'" 60 || exit 1
      python3 "$TOOLS/check_wiring.py" || { echo 'Stopped: the guard could not work like this. Run: stack.sh stop'; exit 1; }
      if ros2 param set /controller_server progress_checker.movement_time_allowance 25.0 2>&1 | grep -q successful; then
        echo '  Nav2 progress checker: 25 s without progress before it gives up (waiting for people is not being stuck)'
      else
        echo '  note: could not raise Nav2 progress_checker.movement_time_allowance; the guard avoids its timeout instead'
      fi
    fi
    tab 'T5 rviz' 'rviz2 --ros-args -p use_sim_time:=true'
    tab 'lanes' "python3 $TOOLS/lanes.py show"
    if [ "$mode" = guard ]; then
      tab 'guard' "GUARD_CAMERA=$gcam GUARD_VEHICLES=$gveh python3 $TOOLS/people_guard.py ${label:-guard_last} 2>&1 | tee $HOME/runs/guard_last.log $keep_guard"
      wait_for 'the guard (/scan_nav)' 'timeout 5 ros2 topic echo /scan_nav --once --field header' 60 || exit 1
    fi
    if [ -n "$camera" ]; then
      if python3 -c "import json,sys,urllib.request; h=json.load(urllib.request.urlopen('$EYE_SERVER/health', timeout=3)); print('  eye: detector on the Mac is up (' + h['model'] + ' on ' + h['device'] + ')')" 2>/dev/null; then
        :
      else
        echo "  note: no detector at $EYE_SERVER. Start eye_server.py on the Mac; the eye shows the camera meanwhile."
      fi
      tab 'eye' "EYE_SERVER=$EYE_SERVER python3 $TOOLS/eye.py ${label:-eye_last} 2>&1 | tee $HOME/runs/eye_last.log $keep_eye"
      tab 'eye view' 'ros2 run rqt_image_view rqt_image_view /eye/image_annotated'
    fi
    if [ -n "$traffic" ]; then
      tab 'traffic' "python3 $TOOLS/traffic.py run ${label:-traffic_last} 2>&1 | tee $HOME/runs/traffic_last.log $keep_traffic"
      wait_for 'the vehicles (traffic tab)' 'timeout 8 ros2 topic echo /traffic/truth --once --field data' 60 || exit 1
    fi
    if [ -n "$clean" ]; then
      # v10.3: the cleaning behind the guard. -r /cmd_vel:=/cmd_vel_raw: coverage_nav2's rescue
      # (reverse 2.5 s) goes through guard_gate and the Collision Monitor, not straight to the wheels
      # v10.4: on simulation time (its stall timer and deadlines are robot time, like Nav2's)
      tab 'clean' "PYTHONUNBUFFERED=1 ros2 run warehouse_planner coverage_nav2 --ros-args -p use_sim_time:=true -p auto_start:=false -r /cmd_vel:=/cmd_vel_raw 2>&1 | tee $HOME/runs/clean_last.log $keep_clean"
      wait_for 'the cleaning (coverage_nav2: /mission_state)' 'timeout 8 ros2 topic echo /mission_state --once --field data' 90 || exit 1
      tab 'coverage rec' "ros2 run warehouse_planner coverage_recorder --ros-args -p use_sim_time:=true -p clean_width:=0.64"
      # v10.4: the robot says what it is doing, as it happens (from /mission_state)
      keep_say=''
      [ -n "$label" ] && keep_say="$HOME/runs/logs/say_$label.log"
      tab 'robot says' "PYTHONUNBUFFERED=1 python3 $TOOLS/narrator.py 2>&1 | tee $HOME/runs/say_last.log $keep_say"
      echo "  cleaning ready: start it with a /mission_command (the run sheet has it), measure it with tour_test.py ${label:-<label>} watch"
    fi
    if [ -n "$talk" ]; then
      OLLAMA="${OLLAMA:-http://${MAC_IP:-10.0.2.2}:11434}"
      if python3 -c "import urllib.request; urllib.request.urlopen('$OLLAMA/api/tags', timeout=3)" 2>/dev/null; then
        echo "  talk: Ollama on the Mac is up ($OLLAMA)"
      else
        echo "  note: no Ollama at $OLLAMA. On the Mac: OLLAMA_HOST=0.0.0.0 ollama serve (and ollama pull llama3.2)."
      fi
      tab 'talk' "ros2 run warehouse_planner llm_commander --ros-args -p ollama_host:=$OLLAMA"
    fi
    if [ -n "$label" ] && [ -z "$scene" ] && [ -z "$traffic" ]; then
      sleep 3
      tab "T6 probe $label" "python3 $TOOLS/worker_clearance.py $label"
    fi
    echo 'Stack is up.'
    ;;
  *)
    echo 'usage: stack.sh stop | stack.sh start [label] [guard] [camera] [scene] [nocam] [traffic] [headless] [novehicles] [clean] [talk]'
    exit 1
    ;;
esac
