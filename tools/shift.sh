#!/usr/bin/env bash
# shift.sh (v10.9, 14 Oct) - one command for a recorded cleaning shift, from the checks to the report.
#
#   shift.sh run <label> [zone ...]      check (doctor.py), stop the last run, start the stack, record,
#                                        clean (default: aisle_1 aisle_2), watch it to the end, stop,
#                                        report (~/runs/report_<label>.txt) and pack it for Claude
#                                        (~/runs/<label>_for_claude.tgz). Always ends with a report:
#                                        the watch gives up after 1800 s of sim time or 90 min of
#                                        real time, whichever comes first.
#   shift.sh report <label> [earlier ...] the report again (lane_report, tour_report, tour_why, the
#                                        log lines that matter), side by side with earlier labels
#   shift.sh send <label>                serve the pack to the Mac: open the address it prints in
#                                        Safari, then Ctrl+C here
#   shift.sh demo [label] [nogui|window] for showing it: the checks with Ollama, then the stack with
#                                        the talk tab, the web page (Mission Control: the map, what the
#                                        robot sees and decides, talk and STOP; a 'web' tab runs
#                                        robot_web.py) and the Gazebo window opened onto the running
#                                        simulation (nogui: none; window: Gazebo started with its own
#                                        window instead, as before v10, if the first way shows none).
#                                        Nothing moves until you tell it to
#   shift.sh gui                         the Gazebo window, on a stack that is running (ign gazebo -g:
#                                        closing it leaves the simulation running)
#   shift.sh web [replay <label>]        the web page alone: on a running stack, or (replay) a recorded
#                                        shift played back through the people guard, no stack needed
#   shift.sh stop                        stop everything (stack.sh stop, the recorder/watcher, the web page)
#
# Every step says what it does; a step that fails says what to do and stops (the stack is stopped).
# no "set -u": the ROS setup scripts read unset variables
TOOLS="$HOME/ros2_ws/src/tools"
RUNS="$HOME/runs"
WALL_MAX=5400            # s of real time the watch may take, at most (90 min)
SIM_MAX=1800             # s of sim time the watch may take, at most

ros() { source /opt/ros/humble/setup.bash >/dev/null 2>&1; source "$HOME/ros2_ws/install/setup.bash" >/dev/null 2>&1; }
step() { printf '\n== %s ==\n' "$*"; }
tab() {  # title, command (as stack.sh opens its tabs)
  gnome-terminal --tab -- bash -ic "printf '\\033]0;$1\\007'; cd $HOME/ros2_ws && source install/setup.bash && $2; exec bash"
}
vm_ip() { hostname -I 2>/dev/null | awk '{print $1}'; }

web_up() {  # wait (up to 30 s) for the web page; say where to open it
  local i
  for i in $(seq 1 15); do
    if python3 -c "import sys, urllib.request as u; sys.exit(0 if b'\"ok\": true' in u.urlopen('http://localhost:8080/api/health', timeout=2).read() else 1)" 2>/dev/null; then
      echo "  the web page is up. On the Mac, open:  http://$(vm_ip):8080"
      echo "  for the microphone, once on the Mac:  curl -o mac_web.py http://$(vm_ip):8080/mac.py"
      echo "                    then each time:     python3 mac_web.py   and open http://localhost:8080"
      return 0
    fi
    sleep 2
  done
  echo "  WARNING: the web page did not answer on port 8080 in 30 s: look at the 'web' tab"
  return 1
}

gui() {  # the Gazebo window on the running simulation
  if ! pgrep -f "ign gazebo.*-s" >/dev/null && ! pgrep -f "gz sim.*-s" >/dev/null; then
    if pgrep -f "ign gazebo" >/dev/null; then echo "  Gazebo already has its window (the stack was started without headless)"; return 0; fi
    echo "  no simulation is running: start one first (shift.sh demo)"; return 1
  fi
  if pgrep -f "ign gazebo -g" >/dev/null; then echo "  the Gazebo window is open already"; return 0; fi
  tab 'gazebo window' "export IGN_GAZEBO_RESOURCE_PATH=\$IGN_GAZEBO_RESOURCE_PATH:\$HOME/ros2_ws/install/warehouse_cleaning_robot/share; ign gazebo -g"
  echo "  the Gazebo window is opening (10-30 s). It costs about two CPU cores: if the page's header shows"
  echo "  the simulation below 0.5x real time and that bothers you, just close the window; the robot carries on."
}

report() {  # label [earlier labels]
  local L="$1"; shift
  local out="$RUNS/report_$L.txt"
  {
    echo "=== shift report: $L ($(date '+%d %b %H:%M')) ==="
    grep -m1 "^VERSION" "$TOOLS/people_guard.py" | sed 's/^/people_guard /'
    grep -m1 -o "UNSTUCK AND IN ORDER (v10.7" "$HOME/ros2_ws/src/warehouse_planner/warehouse_planner/coverage_nav2.py" | sed 's/^/coverage_nav2: /'
    echo
    python3 "$TOOLS/lane_report.py" "$@" "$L" 2>&1
    echo
    [ -f "$TOOLS/tour_report.py" ] && python3 "$TOOLS/tour_report.py" "$@" "$L" 2>&1 | head -150
    echo
    [ -f "$TOOLS/tour_why.py" ] && python3 "$TOOLS/tour_why.py" "$L" 2>&1 | tail -60
    echo
    echo "--- the cleaning's own log (key lines) ---"
    grep -aE "COMMAND|BYPASS|DETOUR|PATCH|DEFER|REVISIT|SKIP|STALL|RESCUE|UNSTICK|ALIGN|TIMEOUT|REJECTED|VEHICLE|CLEARANCE|SAFETY|NAV2|SIM PAUSED|retry|COMPLETE|lanes swept|traffic:|froze" \
      "$RUNS/logs/clean_$L.log" 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | tail -100
    echo
    echo "--- what the robot said (last 40 lines) ---"
    tail -40 "$RUNS/logs/say_$L.log" 2>/dev/null
    echo
    echo "--- the recorder ---"
    tail -3 "$RUNS/logs/rec_$L.log" 2>/dev/null
  } > "$out" 2>&1
  cat "$out"
  echo
  echo "Saved: $out"
}

pack() {  # label
  local L="$1"
  ( cd "$RUNS" && tar -czf "${L}_for_claude.tgz" --ignore-failed-read --exclude="*.~*~" --exclude=__pycache__ \
      "rec/$L.npz" "traffic/$L.csv" "guard/$L.csv" "tour/$L.csv" "tour/${L}_laps.csv" "load/$L.csv" \
      "eye/$L.csv" logs/*_"$L".log "cov_$L" "report_$L.txt" 2>/dev/null )
  ls -lh "$RUNS/${L}_for_claude.tgz" 2>/dev/null | awk '{print "Packed: ~/runs/" $NF " (" $5 ")"}' | sed "s#$RUNS/##"
}

stop_all() {
  pkill -INT -f "scan[_]rec.py" 2>/dev/null
  pkill -INT -f "tour[_]test.py" 2>/dev/null
  pkill -INT -f "robot[_]web.py" 2>/dev/null
  "$TOOLS/stack.sh" stop
}

case "${1:-}" in
  run)
    L="${2:-}"
    if [ -z "$L" ]; then echo "usage: shift.sh run <label> [zone ...]   e.g. shift.sh run shift_v108a"; exit 1; fi
    shift 2
    ZONES=("$@"); [ ${#ZONES[@]} -eq 0 ] && ZONES=(aisle_1 aisle_2)
    for z in "${ZONES[@]}"; do
      case "$z" in front|aisle_1|aisle_2|aisle_3|back) ;; *) echo "Unknown zone '$z' (front aisle_1 aisle_2 aisle_3 back)"; exit 1 ;; esac
    done
    ros
    mkdir -p "$RUNS/logs"
    step "1/7 checks"
    python3 "$TOOLS/doctor.py" "$L" --fix || { echo "NOT STARTED: fix the FAIL line(s) above, then run this again."; exit 1; }
    step "2/7 stopping anything left from the last run; a clean floor"
    stop_all
    rm -f "$HOME/.ros/warehouse_cleaned.json"*
    touch "$RUNS/.shift_${L}_start"
    step "3/7 starting the stack ($L)"
    if ! "$TOOLS/stack.sh" start "$L" guard camera traffic headless clean; then
      echo "NOT STARTED: the stack did not come up (the line above says which part). Stopping it."
      "$TOOLS/stack.sh" stop; exit 1
    fi
    python3 "$TOOLS/doctor.py" --rtf | grep -E "OK|WARN"
    step "4/7 recording, and the command: clean ${ZONES[*]}"
    nohup python3 "$TOOLS/scan_rec.py" "$L" 2400 > "$RUNS/logs/rec_$L.log" 2>&1 &
    REC=$!
    sleep 15
    ZJ=$(printf '"%s", ' "${ZONES[@]}"); ZJ="[${ZJ%, }]"
    ARG="{data: '{\"action\": \"clean\", \"zones\": $ZJ}'}"
    for try in 1 2; do
      timeout 30 ros2 topic pub --once /mission_command std_msgs/msg/String "$ARG" >> "$RUNS/logs/cmd_$L.log" 2>&1
      for i in $(seq 1 10); do
        grep -aq "COMMAND: " "$RUNS/logs/clean_$L.log" 2>/dev/null && break 2
        sleep 2
      done
      echo "  the cleaning did not take the command yet: sending it again"
    done
    if grep -aq "COMMAND: " "$RUNS/logs/clean_$L.log" 2>/dev/null; then
      echo "  the cleaning took it: $(grep -a -m1 "COMMAND: " "$RUNS/logs/clean_$L.log" | sed 's/\x1b\[[0-9;]*m//g; s/.*COMMAND: /COMMAND: /')"
    elif grep -aq "COMMAND REJECTED" "$RUNS/logs/clean_$L.log" 2>/dev/null; then
      echo "  WARNING: the cleaning REJECTED the command: $(grep -a -m1 "COMMAND REJECTED" "$RUNS/logs/clean_$L.log" | sed 's/\x1b\[[0-9;]*m//g; s/.*COMMAND REJECTED/COMMAND REJECTED/')"
    else
      echo "  WARNING: no COMMAND line in the clean tab's log; the watch runs anyway (look at the clean tab)"
    fi
    step "5/7 watching the cleaning to the end (it says 'coverage_nav2 says done'; at most $SIM_MAX s of sim time or $((WALL_MAX / 60)) min)"
    timeout -s INT -k 60 "$WALL_MAX" python3 "$TOOLS/tour_test.py" "$L" watch "$SIM_MAX"   # INT: it saves, as on Ctrl+C
    [ $? -eq 124 ] && echo "  the watch reached $((WALL_MAX / 60)) min of real time: stopping here, the report says how far it got"
    step "6/7 stopping (the recorder saves first)"
    for i in $(seq 1 45); do kill -0 "$REC" 2>/dev/null || break; sleep 2; done
    if kill -0 "$REC" 2>/dev/null; then kill -INT "$REC" 2>/dev/null; for i in $(seq 1 30); do kill -0 "$REC" 2>/dev/null || break; sleep 1; done; fi
    tail -2 "$RUNS/logs/rec_$L.log"
    sleep 5
    "$TOOLS/stack.sh" stop
    sleep 3
    LATEST=$(readlink -f "$RUNS/latest" 2>/dev/null)
    if [ -n "$LATEST" ] && [ -d "$LATEST" ] && [ "$LATEST" -nt "$RUNS/.shift_${L}_start" ] && [ ! -e "$RUNS/cov_$L" ]; then
      mv "$LATEST" "$RUNS/cov_$L" && echo "  coverage saved: ~/runs/cov_$L"
    else
      echo "  note: no new coverage folder in ~/runs/latest (the recorder tab may have failed)"
    fi
    rm -f "$RUNS/.shift_${L}_start"
    step "7/7 the report"
    report "$L"
    pack "$L"
    echo
    echo "DONE. To send it to Claude:  ~/ros2_ws/src/tools/shift.sh send $L"
    ;;
  report)
    L="${2:-}"
    if [ -z "$L" ]; then echo "usage: shift.sh report <label> [earlier label ...]"; exit 1; fi
    shift 2
    report "$L" "$@"
    pack "$L"
    ;;
  send)
    L="${2:-}"
    if [ -z "$L" ] || [ ! -f "$RUNS/${L}_for_claude.tgz" ]; then
      echo "usage: shift.sh send <label>   (needs ~/runs/<label>_for_claude.tgz: shift.sh report <label> makes it)"; exit 1
    fi
    for ip in $(hostname -I); do echo "On the Mac, open in Safari:  http://$ip:8000/${L}_for_claude.tgz"; done
    echo "(then attach the file Safari saved; Ctrl+C here when it is done)"
    cd "$RUNS" && python3 -m http.server 8000
    ;;
  demo)
    L=''; nogui=''; window=''
    for a in "${@:2}"; do
      case "$a" in nogui) nogui=1 ;; window) window=1 ;; *) L="$a" ;; esac
    done
    L="${L:-demo_$(date +%d%b_%H%M)}"
    ros
    step "checks (with Ollama for the talk tab)"
    python3 "$TOOLS/doctor.py" "$L" --talk --fix || { echo "NOT STARTED: fix the FAIL line(s) above, then run this again."; exit 1; }
    step "stopping anything left; a clean floor"
    stop_all
    rm -f "$HOME/.ros/warehouse_cleaned.json"*
    step "starting the stack with the talk tab ($L)"
    HL=headless; [ -n "$window" ] && HL=''
    if ! "$TOOLS/stack.sh" start "$L" guard camera traffic $HL clean talk; then
      echo "NOT STARTED: the stack did not come up (the line above says which part). Stopping it."
      "$TOOLS/stack.sh" stop; exit 1
    fi
    python3 "$TOOLS/doctor.py" --rtf | grep -E "OK|WARN"
    step "the web page (Mission Control) in a 'web' tab"
    tab 'web' "python3 $TOOLS/robot_web.py 2>&1 | tee $RUNS/logs/web_$L.log"
    web_up
    if [ -z "$nogui" ] && [ -z "$window" ]; then
      step "the Gazebo window"
      gui
    fi
    echo
    echo "Ready. Nothing moves until you tell it to: on the web page (or in the talk tab) say or type"
    echo "'clean aisle 1', then 'why did you stop?', 'how much is left?', STOP, 'carry on', 'go home';"
    echo "or click a zone on the page's map."
    echo "When done: ~/ros2_ws/src/tools/shift.sh stop"
    ;;
  gui)
    ros
    gui
    ;;
  web)
    ros
    if pgrep -f "robot[_]web.py" >/dev/null; then
      echo "the web page is running already (its tab, or: pkill -f robot_web.py)"; exit 1
    fi
    mkdir -p "$RUNS/logs"
    if [ "${2:-}" = replay ]; then
      L="${3:-}"
      if [ -z "$L" ] || [ ! -f "$RUNS/rec/$L.npz" ]; then
        echo "usage: shift.sh web replay <label>   (needs ~/runs/rec/<label>.npz: the shifts you ran)"; ls "$RUNS"/rec/*.npz 2>/dev/null | sed 's#.*/##; s#\.npz##; s#^#  #'; exit 1
      fi
      tab 'web (replay)' "python3 $TOOLS/robot_web.py --replay $RUNS $L 2>&1 | tee $RUNS/logs/web_replay.log"
    else
      tab 'web' "python3 $TOOLS/robot_web.py 2>&1 | tee $RUNS/logs/web_last.log"
    fi
    web_up
    ;;
  stop)
    stop_all
    ;;
  *)
    sed -n '2,29p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
