import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


INF = float("inf")


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def yaw_between(source, target):
    return math.atan2(
        float(target["y"]) - float(source["y"]),
        float(target["x"]) - float(source["x"]),
    )


def make_pose(navigator, frame_id, x, y, yaw, z=0.0):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)

    qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def load_mission(mission_file):
    with open(mission_file, "r", encoding="utf-8") as file:
        mission = yaml.safe_load(file)

    if not mission:
        raise ValueError(f"Mission file is empty: {mission_file}")

    waypoints = mission.get("waypoints", [])
    if not waypoints:
        raise ValueError("Mission file must define at least one waypoint")

    names = []
    for index, waypoint in enumerate(waypoints):
        if "name" not in waypoint:
            raise ValueError(f"Waypoint at index {index} has no name")

        name = str(waypoint["name"])
        if name in names:
            raise ValueError(f"Duplicate waypoint name: {name}")

        names.append(name)

    return mission


def path_length(path):
    if path is None or not path.poses:
        return None

    total = 0.0
    previous = path.poses[0].pose.position

    for stamped_pose in path.poses[1:]:
        current = stamped_pose.pose.position
        total += math.hypot(current.x - previous.x, current.y - previous.y)
        previous = current

    return total


def get_planner_cost(navigator, frame_id, source, target, planner_id):
    yaw = yaw_between(source, target)
    start_pose = make_pose(
        navigator=navigator,
        frame_id=frame_id,
        x=source["x"],
        y=source["y"],
        yaw=yaw,
    )
    goal_pose = make_pose(
        navigator=navigator,
        frame_id=frame_id,
        x=target["x"],
        y=target["y"],
        yaw=yaw,
    )

    path = navigator.getPath(start_pose, goal_pose, planner_id, True)

    return path_length(path)


def matrix_cost(cost_matrix, source_index, target_index):
    value = cost_matrix[source_index][target_index]
    if value is None:
        return INF

    return float(value)


def import_pulp():
    try:
        import pulp
    except ImportError as error:
        raise RuntimeError(
            "PuLP is not installed. Install it in the ROS Python environment "
            "with: python3 -m pip install pulp"
        ) from error

    return pulp


def milp_route_from_first(
    first_index,
    names,
    cost_matrix,
    time_limit_sec,
    gap_rel,
    threads,
):
    pulp = import_pulp()
    count = len(names)

    if count == 1:
        return [names[first_index]], 0.0, [], "Optimal", True

    arcs = [
        (source_index, target_index)
        for source_index in range(count)
        for target_index in range(count)
        if source_index != target_index
        and target_index != first_index
        and cost_matrix[source_index][target_index] is not None
    ]

    problem = pulp.LpProblem(
        f"tb4_open_tsp_start_{names[first_index]}",
        pulp.LpMinimize,
    )

    x = {
        arc: pulp.LpVariable(f"x_{arc[0]}_{arc[1]}", cat=pulp.LpBinary)
        for arc in arcs
    }
    order = {
        index: pulp.LpVariable(
            f"u_{index}",
            lowBound=0,
            upBound=count - 1,
            cat=pulp.LpInteger,
        )
        for index in range(count)
    }

    problem += pulp.lpSum(
        float(cost_matrix[source_index][target_index]) * x[(source_index, target_index)]
        for source_index, target_index in arcs
    )

    problem += order[first_index] == 0

    for index in range(count):
        if index == first_index:
            continue

        problem += order[index] >= 1
        problem += pulp.lpSum(
            x[(source_index, target_index)]
            for source_index, target_index in arcs
            if target_index == index
        ) == 1

    problem += pulp.lpSum(
        x[(source_index, target_index)]
        for source_index, target_index in arcs
        if source_index == first_index
    ) == 1

    for index in range(count):
        if index == first_index:
            continue

        problem += pulp.lpSum(
            x[(source_index, target_index)]
            for source_index, target_index in arcs
            if source_index == index
        ) <= 1

    problem += pulp.lpSum(x.values()) == count - 1

    for source_index, target_index in arcs:
        problem += (
            order[target_index]
            >= order[source_index] + 1 - count * (1 - x[(source_index, target_index)])
        )

    solver_kwargs = {
        "msg": False,
        "timeLimit": float(time_limit_sec),
        "gapRel": float(gap_rel),
    }
    if int(threads) > 0:
        solver_kwargs["threads"] = int(threads)

    solver = pulp.PULP_CBC_CMD(**solver_kwargs)
    status_code = problem.solve(solver)
    status = pulp.LpStatus.get(status_code, str(status_code))
    optimality_proven = status == "Optimal" and float(gap_rel) == 0.0

    next_by_source = {}
    for (source_index, target_index), variable in x.items():
        value = pulp.value(variable)
        if value is not None and value > 0.5:
            next_by_source[source_index] = target_index

    route_indexes = [first_index]
    current_index = first_index
    visited = {first_index}

    while current_index in next_by_source:
        current_index = next_by_source[current_index]

        if current_index in visited:
            return [], INF, [], status, optimality_proven

        route_indexes.append(current_index)
        visited.add(current_index)

    if len(route_indexes) != count or set(route_indexes) != set(range(count)):
        return [], INF, [], status, optimality_proven

    route_cost = 0.0
    edge_costs = []

    for source_index, target_index in zip(route_indexes, route_indexes[1:]):
        edge_cost = matrix_cost(cost_matrix, source_index, target_index)
        if math.isinf(edge_cost):
            return [], INF, [], status, optimality_proven

        route_cost += edge_cost
        edge_costs.append(round(edge_cost, 4))

    route_names = [names[index] for index in route_indexes]
    return route_names, route_cost, edge_costs, status, optimality_proven


def routes_document(
    planner_id,
    names,
    routes_by_first,
    route_solver,
    route_time_limit_sec,
    milp_gap_rel,
):
    return {
        "planner_id": planner_id,
        "names": names,
        "route_solver": route_solver,
        "route_time_limit_sec": route_time_limit_sec,
        "milp_gap_rel": milp_gap_rel,
        "routes_by_first": routes_by_first,
    }


def write_route_file(output_file, document):
    output_dir = os.path.dirname(os.path.abspath(output_file))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    with open(output_file, "w", encoding="utf-8") as file:
        yaml.safe_dump(document, file, sort_keys=False)


def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.declare_parameter("mission_file", "")
    navigator.declare_parameter("output_file", "")
    navigator.declare_parameter("planner_id", "")
    navigator.declare_parameter("wait_for_nav2_active", True)
    navigator.declare_parameter("request_delay_sec", 0.0)
    navigator.declare_parameter("route_time_limit_sec", 60.0)
    navigator.declare_parameter("milp_gap_rel", 0.0)
    navigator.declare_parameter("milp_threads", 0)

    mission_file = navigator.get_parameter("mission_file").value
    output_file = navigator.get_parameter("output_file").value
    planner_id = navigator.get_parameter("planner_id").value
    wait_for_nav2_active = navigator.get_parameter("wait_for_nav2_active").value
    request_delay_sec = navigator.get_parameter("request_delay_sec").value
    route_time_limit_sec = navigator.get_parameter("route_time_limit_sec").value
    milp_gap_rel = navigator.get_parameter("milp_gap_rel").value
    milp_threads = navigator.get_parameter("milp_threads").value

    if not mission_file:
        navigator.get_logger().error("Parameter mission_file is required")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    if not output_file:
        root, _extension = os.path.splitext(mission_file)
        output_file = f"{root}_routes.yaml"

    try:
        import_pulp()
    except RuntimeError as error:
        navigator.get_logger().error(str(error))
        navigator.destroy_node()
        rclpy.shutdown()
        return

    try:
        mission = load_mission(mission_file)
    except Exception as error:
        navigator.get_logger().error(str(error))
        navigator.destroy_node()
        rclpy.shutdown()
        return

    frame_id = mission.get("frame_id", "map")
    waypoints = mission["waypoints"]
    count = len(waypoints)
    cost_matrix = [[None for _ in range(count)] for _ in range(count)]

    if wait_for_nav2_active:
        navigator.get_logger().info("Waiting for Nav2 to become active...")
        navigator.waitUntilNav2Active()

    navigator.get_logger().info(
        f"Computing planner cost matrix for {count} waypoint(s)"
    )

    for source_index, source in enumerate(waypoints):
        source_name = source["name"]

        for target_index, target in enumerate(waypoints):
            target_name = target["name"]

            if source_index == target_index:
                cost_matrix[source_index][target_index] = 0.0
                continue

            navigator.get_logger().info(
                f"Planning {source_name} -> {target_name} "
                f"({source_index + 1}/{count}, {target_index + 1}/{count})"
            )

            try:
                cost = get_planner_cost(
                    navigator=navigator,
                    frame_id=frame_id,
                    source=source,
                    target=target,
                    planner_id=planner_id,
                )
            except Exception as error:
                navigator.get_logger().error(
                    f"Planner failed for {source_name} -> {target_name}: {error}"
                )
                cost = None

            if cost is None:
                navigator.get_logger().warn(
                    f"No valid path for {source_name} -> {target_name}"
                )
            else:
                cost_matrix[source_index][target_index] = round(float(cost), 4)

            if request_delay_sec > 0.0:
                time.sleep(float(request_delay_sec))

    names = [str(waypoint["name"]) for waypoint in waypoints]
    routes_by_first = {}

    navigator.get_logger().info(
        f"Computing MILP offline route for each first waypoint "
        f"with {float(route_time_limit_sec):.1f} s per route, "
        f"gap_rel={float(milp_gap_rel):.6f}"
    )
    for first_index, first_name in enumerate(names):
        navigator.get_logger().info(
            f"Computing MILP route starting from {first_name} "
            f"({first_index + 1}/{count})"
        )

        (
            route,
            route_cost,
            edge_costs,
            route_status,
            optimality_proven,
        ) = milp_route_from_first(
            first_index=first_index,
            names=names,
            cost_matrix=cost_matrix,
            time_limit_sec=float(route_time_limit_sec),
            gap_rel=float(milp_gap_rel),
            threads=int(milp_threads),
        )

        if not route or math.isinf(route_cost):
            navigator.get_logger().warn(f"No complete route starting from {first_name}")
            routes_by_first[first_name] = {
                "route": [],
                "cost": None,
                "edge_costs": [],
                "solver_status": route_status,
                "optimality_proven": False,
            }
        else:
            routes_by_first[first_name] = {
                "route": route,
                "cost": round(float(route_cost), 4),
                "edge_costs": edge_costs,
                "solver_status": route_status,
                "optimality_proven": bool(optimality_proven),
            }

        document = routes_document(
            planner_id=planner_id,
            names=names,
            routes_by_first=routes_by_first,
            route_solver="pulp_cbc_milp",
            route_time_limit_sec=float(route_time_limit_sec),
            milp_gap_rel=float(milp_gap_rel),
        )
        write_route_file(output_file, document)

    navigator.get_logger().info(f"Offline routes written to {output_file}")
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
