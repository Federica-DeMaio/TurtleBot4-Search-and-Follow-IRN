import os
import time

import rclpy
import yaml
from nav2_simple_commander.robot_navigator import BasicNavigator

from .planner_cost_matrix import get_planner_cost, load_mission


def load_yaml_file(path):
    with open(path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    if not data:
        raise ValueError(f"YAML file is empty: {path}")

    return data


def write_yaml_file(path, data):
    output_dir = os.path.dirname(os.path.abspath(path))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    with open(path, "w", encoding="utf-8") as file:
        yaml.safe_dump(data, file, sort_keys=False)


def waypoints_by_name(mission):
    waypoints = {}

    for waypoint in mission.get("waypoints", []):
        if "name" not in waypoint:
            raise ValueError("Each waypoint must define a name")

        name = str(waypoint["name"])
        if name in waypoints:
            raise ValueError(f"Duplicate waypoint name: {name}")

        waypoints[name] = dict(waypoint)

    if not waypoints:
        raise ValueError("Mission file must define at least one waypoint")

    return waypoints


def normalize_route(route_info):
    route = route_info.get("route", [])
    if route is None:
        route = []

    return [str(name) for name in route]


def recompute_route_cost(
    navigator,
    frame_id,
    waypoint_lookup,
    route,
    planner_id,
    round_digits,
):
    if not route:
        return None, []

    unknown_names = [
        name for name in route
        if name not in waypoint_lookup
    ]
    if unknown_names:
        raise ValueError(
            "Route contains waypoint(s) missing from mission file: "
            + ", ".join(unknown_names)
        )

    if len(route) == 1:
        return 0.0, []

    edge_costs = []

    for source_name, target_name in zip(route, route[1:]):
        source = waypoint_lookup[source_name]
        target = waypoint_lookup[target_name]

        cost = get_planner_cost(
            navigator=navigator,
            frame_id=frame_id,
            source=source,
            target=target,
            planner_id=planner_id,
        )

        if cost is None:
            return None, []

        edge_costs.append(round(float(cost), int(round_digits)))

    total_cost = round(sum(edge_costs), int(round_digits))
    return total_cost, edge_costs


def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.declare_parameter("mission_file", "")
    navigator.declare_parameter("route_file", "")
    navigator.declare_parameter("output_file", "")
    navigator.declare_parameter("planner_id", "")
    navigator.declare_parameter("wait_for_nav2_active", False)
    navigator.declare_parameter("request_delay_sec", 0.0)
    navigator.declare_parameter("round_digits", 4)

    mission_file = navigator.get_parameter("mission_file").value
    route_file = navigator.get_parameter("route_file").value
    output_file = navigator.get_parameter("output_file").value
    planner_id = navigator.get_parameter("planner_id").value
    wait_for_nav2_active = navigator.get_parameter("wait_for_nav2_active").value
    request_delay_sec = navigator.get_parameter("request_delay_sec").value
    round_digits = navigator.get_parameter("round_digits").value

    if not mission_file:
        navigator.get_logger().error("Parameter mission_file is required")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    if not route_file:
        navigator.get_logger().error("Parameter route_file is required")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    if not output_file:
        output_file = route_file

    try:
        mission = load_mission(mission_file)
        route_data = load_yaml_file(route_file)
        waypoint_lookup = waypoints_by_name(mission)
    except Exception as error:
        navigator.get_logger().error(str(error))
        navigator.destroy_node()
        rclpy.shutdown()
        return

    routes_by_first = route_data.get("routes_by_first")
    if not routes_by_first:
        navigator.get_logger().error("Route file must define routes_by_first")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    frame_id = mission.get("frame_id", "map")
    route_planner_id = str(route_data.get("planner_id", ""))
    if not planner_id:
        planner_id = route_planner_id

    if not planner_id:
        navigator.get_logger().warn(
            "No planner_id specified; Nav2 will use its default planner"
        )

    if wait_for_nav2_active:
        navigator.get_logger().info("Waiting for Nav2 to become active...")
        navigator.waitUntilNav2Active()

    navigator.get_logger().info(
        f"Recomputing route costs using planner_id='{planner_id}'"
    )

    for first_name, route_info in routes_by_first.items():
        route = normalize_route(route_info)
        route_info["route"] = route

        navigator.get_logger().info(
            f"Recomputing route from {first_name}: {len(route)} waypoint(s)"
        )

        try:
            total_cost, edge_costs = recompute_route_cost(
                navigator=navigator,
                frame_id=frame_id,
                waypoint_lookup=waypoint_lookup,
                route=route,
                planner_id=planner_id,
                round_digits=round_digits,
            )
        except Exception as error:
            navigator.get_logger().error(
                f"Failed to recompute route from {first_name}: {error}"
            )
            total_cost = None
            edge_costs = []

        route_info["cost"] = total_cost
        route_info["edge_costs"] = edge_costs

        if total_cost is None:
            navigator.get_logger().warn(f"Route from {first_name} has no valid cost")
        else:
            navigator.get_logger().info(
                f"Route from {first_name} cost: {total_cost:.4f} m"
            )

        if request_delay_sec > 0.0:
            time.sleep(float(request_delay_sec))

    if planner_id:
        route_data["planner_id"] = str(planner_id)

    write_yaml_file(output_file, route_data)
    navigator.get_logger().info(f"Updated route costs written to {output_file}")

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
