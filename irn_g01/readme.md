# irn_g01

ROS 2 Python package with the ArUco detection and target-following nodes used by
the IRN TurtleBot4 project.

## Nodes

- `aruco_reader`: detects the configured ArUco marker and publishes
  `/aruco/pose`.
- `compute_pose`: transforms visible marker poses into map-frame robot goals and
  publishes predicted goals when the marker is lost.
- `BT_FSM_follow`: finite state machine that sends visible or predicted goals to
  Nav2 and handles lost-target recovery.

## Python dependencies

Install the non-ROS Python dependencies from the package root:

```bash
python3 -m pip install -r requirements.txt
```

Build from a ROS 2 workspace with:

```bash
colcon build --packages-select irn_g01
```
