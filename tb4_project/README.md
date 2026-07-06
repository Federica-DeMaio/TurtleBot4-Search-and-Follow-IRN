# tb4_project

ROS 2 package with the TurtleBot4 launch files, Nav2 configuration, Behavior
Trees and waypoint mission tools used by the IRN project.

## Main launch files

- `full_real.launch.py`: starts localization, RViz, ArUco detection, Nav2,
  follow and waypoint search as a single real-robot pipeline.
- `real_search_mission.launch.py`: runs the waypoint search mission.
- `follow.launch.py`: runs the ArUco follow pipeline.
- `aruco_reader.launch.py`: starts the marker detector from `irn_g01`.

## Tools

- `search_mission`: visits offline-optimized waypoint routes until the marker is
  detected.
- `compute_cost_matrix`: computes Nav2 path costs and offline MILP routes.
- `recompute_route_costs`: refreshes route costs for an existing route file.

Build from a ROS 2 workspace with:

```bash
colcon build --packages-select tb4_project
```
