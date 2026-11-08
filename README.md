# Autonomous warehouse cleaning robot (ROS 2, simulation)

A cleaning robot that sweeps warehouse aisles while sharing them with people and forklifts. It predicts where people are walking, waits outside their path, explains why it stopped, and afterwards replays every stop to check whether it was needed.

Simulation only (Gazebo Fortress); no hardware yet.

## What it does
- **Cleans** the aisles lane by lane: a boustrophedon coverage planner driving Nav2 (NavFn + Regulated Pure Pursuit).
- **People guard** (`tools/people_guard.py`): tracks people and vehicles from the 2D lidar, predicts their paths every 0.1 s, waits at a stop line outside their walkway and slows near people. A fail-safe gate with an emergency stop sits between Nav2 and the wheels.
- **Camera:** YOLOv8n confirms people standing still.
- **Talk to it:** plain-English commands ("clean aisle 1", "why did you stop?") through a local LLM (llama3.2 with Ollama, LangGraph). It can only choose from checked commands; every motion decision is plain code.
- **Mission Control** (`tools/robot_web.py`): a web page with the live map, what the robot sees and why it acts, plus a 3D view (`/twin`, three.js).
- **Rewind** (`tools/rewind.py`): for every stop for a person, the future it avoided: where it would have driven, against where that person really walked.
- **Wet-floor guardian** (`tools/wet_floor.py`): turns the water off where people walk and warns anyone heading onto wet floor.

## Results (5 recorded shifts, replayed through the real code)
- 172 stops for people: up to 78 would have ended in contact (the simulated people don't dodge, so a worst case); 53 weren't needed.
- 93 % less time with people on freshly wet floor (assuming it dries in 2 minutes).
- No false person tracks in 25,769 lidar scans; walking people in sight within 2 m tracked 88 % of the time.

## Layout
- `warehouse_cleaning_robot/`: the robot model, the warehouse world, launch files and Nav2 settings
- `warehouse_planner/`: the coverage planner, the LLM agent and the command node
- `tools/`: the people guard, Mission Control and the 3D view, Rewind, the wet-floor guardian, the start-up scripts (`stack.sh`, `shift.sh`) and reports
- `tools/lab/`: tests, replays and audits

## Running it
Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, Nav2. Build with `colcon build`, then run `tools/shift.sh demo` and open port 8080 (the 3D view is at `/twin`). Without the simulator, `tools/shift.sh web replay <label>` plays back a recorded shift. Written for my own setup (an Ubuntu VM in UTM on a Mac M3, with YOLO and Ollama on the Mac), so expect to adjust paths.

## Built with
ROS 2 Humble · Gazebo Fortress · Nav2 · Python · YOLOv8n · LangGraph + Ollama (llama3.2) · three.js

By Thomas Antony Kuzhikandam.
