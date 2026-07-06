"""Missione di ricerca per TurtleBot4 reale.

Il nodo aspetta la posa iniziale impostata da RViz, sceglie la route offline
migliore partendo dalla posa reale del robot, visita i waypoint della mappa e
interrompe la ricerca appena il detector ArUco pubblica una posa valida.
"""

import math
import os
import time
from datetime import datetime, timezone

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import Spin
from nav2_msgs.srv import GetCostmap
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


INF = float("inf")
TWO_PI = 2.0 * math.pi
VISIT_STATE_FORMAT = "tb4_project_waypoint_visit_state_v1"
FIRST_WAYPOINT_START_ONLY_MISSIONS = {"diem_waypoints_reduced.yaml"}


def first_waypoint_cost_policy(mission_file):
    """Sceglie come valutare il primo waypoint in base al file missione."""
    mission_name = os.path.basename(str(mission_file))
    if mission_name in FIRST_WAYPOINT_START_ONLY_MISSIONS:
        return "start_only"

    return "route_total"

# Questi valori devono restare coerenti con il behavior_server in
# nav2_timeout.yaml. Non comandano direttamente la velocita dello Spin:
# servono a stimare un timeout realistico per la action Spin.
SPIN_MAX_ROTATIONAL_VEL = 0.30
SPIN_ROTATIONAL_ACCEL = 1.0
SPIN_TIME_MARGIN = 2.0
SPIN_MIN_TIME_ALLOWANCE = 3.0


def normalize_angle(angle):
    """Riporta un angolo nell'intervallo [-pi, pi]."""
    # atan2(sin, cos) mantiene la stessa direzione geometrica ma elimina giri
    # interi inutili. Esempio: 3*pi diventa pi, 4.71 diventa circa -1.57.
    return math.atan2(math.sin(angle), math.cos(angle))


def signed_rotation_to_target(current_yaw, target_yaw, requested_angle):
    """Calcola lo Spin relativo rispettando il segno scritto nello YAML."""
    # target_yaw e current_yaw sono orientamenti assoluti in mappa, mentre la
    # action Spin vuole una rotazione relativa da eseguire adesso.
    # delta indica di quanto mi sposto a partire dall'orientamento corrente per
    # raggiungere il target.
    # requested angle indica solo lo scan angle da raggiungere.
    delta = normalize_angle(target_yaw - current_yaw)

    if requested_angle > 0.0:
        # Angolo YAML positivo: vogliamo coprire il settore in senso antiorario.
        # Se delta normalizzato e' negativo, significa che la scorciatoia
        # sarebbe oraria; aggiungiamo 2*pi per forzare il giro antiorario.

        # se il target finale coincide con lo yaw attuale,
        # ma l'angolo richiesto non era zero,
        # allora esegui un giro completo.
        if abs(delta) < 1e-6 and abs(requested_angle) > 1e-6:
            return TWO_PI
        if delta < 0.0:
            return delta + TWO_PI
        return delta

    if requested_angle < 0.0:
        # Angolo YAML negativo: vogliamo coprire il settore in senso orario.
        # Se delta normalizzato e' positivo, sottraiamo 2*pi per evitare che
        # Spin scelga il verso antiorario piu corto.
        if abs(delta) < 1e-6 and abs(requested_angle) > 1e-6:
            return -TWO_PI
        if delta > 0.0:
            return delta - TWO_PI
        return delta

    return delta


def duration_msg(seconds):
    """Converte secondi float nel messaggio Duration richiesto da Spin."""
    seconds = max(0.0, float(seconds))
    whole_seconds = int(seconds)
    nanoseconds = int((seconds - whole_seconds) * 1e9)

    # Le action ROS non accettano un semplice float per i timeout: vogliono un
    # builtin_interfaces/Duration separato in secondi e nanosecondi.
    message = DurationMsg()
    message.sec = whole_seconds
    message.nanosec = nanoseconds
    return message


def spin_time_allowance(angle):
    """Stima il timeout massimo dello Spin dai limiti configurati in Nav2."""
    angle = abs(float(angle))
    if angle < 1e-6:
        return SPIN_MIN_TIME_ALLOWANCE

    # Modello trapezoidale semplice: accelero fino alla velocita massima,
    # eventualmente mantengo velocita costante, poi decelero.
    accel_time = SPIN_MAX_ROTATIONAL_VEL / SPIN_ROTATIONAL_ACCEL
    accel_angle = 0.5 * SPIN_ROTATIONAL_ACCEL * accel_time * accel_time

    if angle >= 2.0 * accel_angle:
        motion_time = (
            2.0 * accel_time
            + (angle - 2.0 * accel_angle) / SPIN_MAX_ROTATIONAL_VEL
        )
    else:
        # Per rotazioni brevi il robot non raggiunge la velocita massima:
        # accelera e poi decelera subito.
        motion_time = 2.0 * math.sqrt(angle / SPIN_ROTATIONAL_ACCEL)

    # Il margine evita falsi timeout per ritardi di action server, TF o carico CPU.
    return max(SPIN_MIN_TIME_ALLOWANCE, motion_time + SPIN_TIME_MARGIN)


def yaw_to_quaternion(yaw):
    """Converte yaw planare in quaternion per PoseStamped/Nav2."""
    # Noi ragioniamo in 2D con un solo angolo yaw. ROS pero rappresenta
    # l'orientamento di una Pose con un quaternion (x, y, z, w). Per una
    # rotazione solo attorno a Z, x e y sono zero; z e w codificano lo yaw.
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def yaw_between(source, target):
    """Calcola la direzione di arrivo dal punto source al punto target."""
    # Questa yaw e' la direzione geometrica del segmento source -> target.
    # La usiamo come orientamento finale del goal Nav2 quando raggiungiamo un
    # waypoint, cosi gli scan_angles restano relativi al verso di arrivo.
    return math.atan2(
        float(target["y"]) - float(source["y"]),
        float(target["x"]) - float(source["x"]),
    )


def quaternion_to_yaw(quaternion):
    """Estrae lo yaw planare da un quaternion ROS."""
    # RViz pubblica /initialpose come quaternion. Per scegliere la route e
    # calcolare le direzioni tra waypoint ci serve invece lo yaw come float.
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def make_pose(navigator, frame_id, x, y, yaw, z=0.0):
    """Crea una PoseStamped per goal Nav2 e richieste al planner."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)

    # Nav2 vuole orientation come quaternion. Qui convertiamo lo yaw calcolato
    # dal nostro codice nel formato richiesto dai messaggi geometry_msgs/Pose.
    qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def load_mission(mission_file):
    """Carica il file missione con waypoint e scan sectors."""
    with open(mission_file, "r", encoding="utf-8") as file:
        mission = yaml.safe_load(file)

    # Senza waypoint non esiste una missione di ricerca eseguibile.
    if not mission:
        raise ValueError(f"Mission file is empty: {mission_file}")

    if "waypoints" not in mission or not mission["waypoints"]:
        raise ValueError("Mission file must define at least one waypoint")

    return mission


def load_route_file(route_file):
    """Carica le route offline prodotte dal calcolo MILP."""
    with open(route_file, "r", encoding="utf-8") as file:
        route_data = yaml.safe_load(file)

    if not route_data:
        raise ValueError(f"Route file is empty: {route_file}")

    names = route_data.get("names", route_data.get("waypoint_names"))
    routes_by_first = route_data.get("routes_by_first")
    planner_id = route_data.get("planner_id", "")

    if not names or not routes_by_first:
        raise ValueError("Route file must define names and routes_by_first")

    # Normalizziamo tutto a stringhe/float per non dipendere da come PyYAML ha
    # interpretato i valori nel file. Da qui in poi i lookup sono stabili.
    names = [str(name) for name in names]
    normalized_routes = {}

    for first_name, route_info in routes_by_first.items():
        # Ogni entry routes_by_first[wp_x] contiene la route ottima assumendo
        # wp_x come primo waypoint dopo la posa iniziale reale.
        route = route_info.get("route", [])
        edge_costs = route_info.get("edge_costs", [])
        cost = route_info.get("cost")

        normalized_routes[str(first_name)] = {
            "route": [str(name) for name in route],
            "edge_costs": [float(value) for value in edge_costs],
            "cost": None if cost is None else float(cost),
            "solver_status": route_info.get("solver_status", ""),
            "optimality_proven": bool(route_info.get("optimality_proven", False)),
        }

    return {
        "names": names,
        "routes_by_first": normalized_routes,
        "planner_id": str(planner_id),
    }


def utc_now_iso():
    """Restituisce un timestamp ISO stabile per il file di ripresa missione."""
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def default_visit_state_file():
    """Path fisso dello stato missione creato quando il launch non lo passa."""
    return os.path.join(
        os.path.expanduser("~"),
        ".ros",
        "tb4_project",
        "waypoint_visitati.yaml",
    )


def normalize_state_file_path(path):
    """Espande ~ e variabili d'ambiente nel path del file waypoint_visitati."""
    if not path:
        return ""

    return os.path.abspath(os.path.expandvars(os.path.expanduser(str(path))))


def load_visit_state(state_file):
    """Carica lo stato di una missione interrotta."""
    with open(state_file, "r", encoding="utf-8") as file:
        state = yaml.safe_load(file)

    if not state:
        raise ValueError(f"Visit state file is empty: {state_file}")

    if state.get("format") != VISIT_STATE_FORMAT:
        raise ValueError(
            f"Unsupported visit state format in {state_file}: {state.get('format')}"
        )

    route = state.get("route", [])
    if not route:
        raise ValueError("Visit state file does not contain a route")

    state["route"] = [str(name) for name in route]
    state["visited_waypoints"] = [
        str(name) for name in state.get("visited_waypoints", [])
    ]
    state["skipped_waypoints"] = [
        str(name) for name in state.get("skipped_waypoints", [])
    ]
    state["processed_waypoints"] = list(state.get("processed_waypoints", []))
    return state


def make_visit_state(mission_file, route_file, planner_id, frame_id, ordered_waypoints):
    """Prepara lo YAML che permette di riprendere la stessa route in seguito."""
    route = [waypoint_name(waypoint) for waypoint in ordered_waypoints]
    return {
        "format": VISIT_STATE_FORMAT,
        "created_at": utc_now_iso(),
        "updated_at": utc_now_iso(),
        "mission_file": os.path.abspath(str(mission_file)),
        "route_file": os.path.abspath(str(route_file)) if route_file else "",
        "planner_id": str(planner_id),
        "frame_id": str(frame_id),
        "route": route,
        "visited_waypoints": [],
        "skipped_waypoints": [],
        "processed_waypoints": [],
        "last_reached_waypoint": None,
        "last_processed_waypoint": None,
    }


def write_visit_state(state_file, state):
    """Scrive su disco lo stato missione aggiornato."""
    state_dir = os.path.dirname(os.path.abspath(state_file))
    if state_dir:
        os.makedirs(state_dir, exist_ok=True)

    state["updated_at"] = utc_now_iso()
    with open(state_file, "w", encoding="utf-8") as file:
        yaml.safe_dump(state, file, sort_keys=False)


def processed_waypoint_names(state):
    """Restituisce tutti i waypoint gia completati o skippati."""
    names = set(str(name) for name in state.get("visited_waypoints", []))
    names.update(str(name) for name in state.get("skipped_waypoints", []))

    for item in state.get("processed_waypoints", []):
        if isinstance(item, dict) and item.get("name"):
            names.add(str(item["name"]))

    return names


def build_ordered_waypoints_from_saved_route(mission, state):
    """Ricostruisce l'ordine route salvato nel file waypoint_visitati."""
    waypoint_by_name = {
        waypoint_name(waypoint): dict(waypoint)
        for waypoint in mission["waypoints"]
    }
    processed_names = processed_waypoint_names(state)
    ordered = []

    for name in state.get("route", []):
        name = str(name)
        if name not in waypoint_by_name:
            raise ValueError(f"Saved route contains waypoint not in mission: {name}")

        if name in processed_names:
            continue

        ordered.append(dict(waypoint_by_name[name]))

    return ordered


def record_waypoint_state(
    state_file,
    state,
    waypoint,
    status,
    reached,
    navigator,
):
    """Aggiorna il file waypoint_visitati dopo un waypoint raggiunto o skippato."""
    if not state_file or state is None:
        return

    name = waypoint_name(waypoint)
    event = {
        "name": name,
        "status": str(status),
        "reached": bool(reached),
        "type": waypoint.get("type", "transit"),
        "timestamp": utc_now_iso(),
    }

    state.setdefault("processed_waypoints", []).append(event)

    if reached:
        visited = state.setdefault("visited_waypoints", [])
        if name not in visited:
            visited.append(name)
        state["last_reached_waypoint"] = name
    else:
        skipped = state.setdefault("skipped_waypoints", [])
        if name not in skipped:
            skipped.append(name)

    state["last_processed_waypoint"] = name
    write_visit_state(state_file, state)
    navigator.get_logger().info(
        f"Updated waypoint visit state: {name} status={status} reached={reached}"
    )


def waypoint_name(waypoint):
    """Legge il nome stabile usato per collegare missione e route."""
    if "name" not in waypoint:
        raise ValueError("Each waypoint must have a name when using offline routes")

    return str(waypoint["name"])


def path_length(path):
    """Somma la lunghezza geometrica di un path Nav2."""
    if path is None or not path.poses:
        return INF

    total = 0.0
    previous = path.poses[0].pose.position

    # Il planner restituisce molti punti intermedi, non solo source e target.
    # Sommiamo la distanza tra punti consecutivi per stimare il costo reale.
    for stamped_pose in path.poses[1:]:
        current = stamped_pose.pose.position
        total += math.hypot(current.x - previous.x, current.y - previous.y)
        previous = current

    return total


def initial_pose_to_start_pose(message):
    """Converte /initialpose nel dizionario usato dalla scelta route."""
    pose = message.pose.pose
    return {
        "x": pose.position.x,
        "y": pose.position.y,
        # /initialpose arriva come PoseWithCovarianceStamped e quindi contiene
        # un quaternion. Per il calcolo yaw_between e per la route selection ci
        # serve una yaw scalare in radianti.
        "yaw": quaternion_to_yaw(pose.orientation),
    }


def initial_pose_to_pose_stamped(navigator, message, default_frame_id):
    """Converte /initialpose nel formato richiesto da BasicNavigator."""
    pose = PoseStamped()
    pose.header.frame_id = message.header.frame_id or default_frame_id
    # Usiamo il clock corrente del nodo per evitare problemi di timestamp
    # vecchi quando Nav2/AMCL trasformano la posa iniziale.
    pose.header.stamp = navigator.get_clock().now().to_msg()
    # Qui manteniamo la posa completa di RViz, incluso il quaternion originale:
    # questa e' la rappresentazione che Nav2 usa per inizializzare AMCL.
    pose.pose = message.pose.pose
    return pose


def wait_for_initial_pose(navigator, topic, frame_id):
    """Aspetta la posa iniziale pubblicata da RViz prima di partire."""
    received_pose = {
        "start_pose": None,
        "stamped_pose": None,
    }

    def callback(message):
        # Salviamo due forme della stessa posa:
        # - dizionario x/y/yaw per ragionamenti geometrici nel codice;
        # - PoseStamped per passarla direttamente a BasicNavigator/AMCL.
        received_pose["start_pose"] = initial_pose_to_start_pose(message)
        received_pose["stamped_pose"] = initial_pose_to_pose_stamped(
            navigator,
            message,
            frame_id,
        )

    subscription = navigator.create_subscription(
        PoseWithCovarianceStamped,
        topic,
        callback,
        10,
    )

    navigator.get_logger().info(f"Waiting for initial pose from RViz on {topic}...")
    while rclpy.ok() and received_pose["start_pose"] is None:
        # Senza spin_once la callback della subscription non verrebbe eseguita.
        rclpy.spin_once(navigator, timeout_sec=0.1)

    navigator.destroy_subscription(subscription)
    return received_pose["start_pose"], received_pose["stamped_pose"]


def is_empty_target_pose(message):
    """Riconosce il messaggio vuoto usato dal detector per 'non trovato'."""
    pose = message.pose
    return (
        pose.position.x == 0.0
        and pose.position.y == 0.0
        and pose.position.z == 0.0
        and pose.orientation.x == 0.0
        and pose.orientation.y == 0.0
        and pose.orientation.z == 0.0
        and pose.orientation.w == 1.0
    )


def create_target_subscription(navigator, topic):
    """Sottoscrive /aruco/pose e aggiorna un flag condiviso target_found."""
    target_state = {
        "found": False,
        "latest_pose": None,
    }

    def callback(message):
        # Il detector puo pubblicare una posa vuota per indicare marker assente.
        # In quel caso non dobbiamo interrompere la missione.
        if is_empty_target_pose(message):
            return

        if not target_state["found"]:
            position = message.pose.position
            navigator.get_logger().info(
                "Target detected from ArUco pose: "
                f"x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}, "
                f"frame={message.header.frame_id}"
            )

        # Appena arriva una posa valida, la search mission deve fermarsi: il
        # follow dell'ArUco verra gestito dagli altri nodi/launch.
        target_state["found"] = True
        target_state["latest_pose"] = message

    subscription = navigator.create_subscription(
        PoseStamped,
        topic,
        callback,
        10,
    )
    navigator.get_logger().info(f"Listening for target detections on {topic}")
    return target_state, subscription


def planned_cost_from_start(navigator, frame_id, start_pose, waypoint, planner_id):
    """Calcola con Nav2 il costo dalla posa iniziale reale a un waypoint."""
    # La route offline contiene gia l'ordine ottimo da ciascun possibile primo
    # waypoint. Online dobbiamo solo decidere quale primo waypoint conviene
    # raggiungere dalla posa iniziale effettiva del robot.
    yaw = yaw_between(start_pose, waypoint)

    # start usa la yaw reale impostata da RViz; goal usa la direzione geometrica
    # start -> waypoint. Questa yaw serve al goal del planner, ma il costo che
    # usiamo e' la lunghezza del path generato.
    start = make_pose(
        navigator=navigator,
        frame_id=frame_id,
        x=start_pose["x"],
        y=start_pose["y"],
        yaw=start_pose["yaw"],
    )
    goal = make_pose(
        navigator=navigator,
        frame_id=frame_id,
        x=waypoint["x"],
        y=waypoint["y"],
        yaw=yaw,
    )

    # getPath chiama planner_server senza muovere il robot. planner_id arriva
    # dal file route, cosi il piano online usa lo stesso planner dell'offline.
    path = navigator.getPath(start, goal, planner_id, True)
    return path_length(path)


def order_waypoints_from_routes(
    navigator,
    frame_id,
    start_pose,
    waypoints,
    route_data,
    planner_id,
    first_waypoint_policy="route_total",
):
    """Sceglie la route offline migliore rispetto alla posa iniziale reale."""
    waypoint_by_name = {waypoint_name(waypoint): dict(waypoint) for waypoint in waypoints}
    mission_names = [waypoint_name(waypoint) for waypoint in waypoints]

    # Missione e file route devono descrivere lo stesso insieme di waypoint.
    # Se manca un nome, la route offline non e' compatibile con la missione.
    missing_names = [
        name for name in mission_names
        if name not in route_data["names"]
    ]
    if missing_names:
        raise ValueError(
            "Route file does not contain waypoint(s): "
            + ", ".join(missing_names)
    )

    start_costs = {}
    for name in mission_names:
        # Per ogni waypoint calcoliamo solo il tratto start reale -> waypoint.
        # Questo e' l'unico calcolo online necessario.
        waypoint = waypoint_by_name[name]
        cost = planned_cost_from_start(
            navigator=navigator,
            frame_id=frame_id,
            start_pose=start_pose,
            waypoint=waypoint,
            planner_id=planner_id,
        )
        start_costs[name] = cost

        if math.isinf(cost):
            navigator.get_logger().warn(f"No valid path from start to {name}")
        else:
            navigator.get_logger().info(f"Planner cost start -> {name}: {cost:.2f} m")

    best_first_name = None
    best_total_cost = INF
    best_route_info = None

    for first_name in mission_names:
        # Costo totale candidato = costo online start -> first_name +
        # costo offline della route che parte da first_name.
        start_cost = float(start_costs.get(first_name, INF))
        route_info = route_data["routes_by_first"].get(first_name)

        if route_info is None:
            navigator.get_logger().warn(f"No offline route starting from {first_name}")
            continue

        route_cost = route_info.get("cost")
        if route_cost is None:
            navigator.get_logger().warn(f"Offline route from {first_name} is invalid")
            continue

        if first_waypoint_policy == "start_only":
            total_cost = start_cost
        else:
            total_cost = start_cost + float(route_cost)

        if total_cost < best_total_cost:
            best_first_name = first_name
            best_total_cost = total_cost
            best_route_info = route_info

    if best_route_info is None:
        raise RuntimeError("Could not select a finite offline route from start pose")

    navigator.get_logger().info(
        f"Selected offline route starting from {best_first_name}, "
        f"selection policy={first_waypoint_policy}, "
        f"estimated selection cost: {best_total_cost:.2f} m"
    )

    route = best_route_info["route"]
    edge_costs = best_route_info.get("edge_costs", [])
    route_names = set(route)
    mission_name_set = set(mission_names)

    # La route selezionata deve visitare tutti e soli i waypoint della missione.
    if route_names != mission_name_set:
        raise ValueError("Selected offline route does not match mission waypoints")

    ordered = []
    current = dict(start_pose)

    for index, name in enumerate(route):
        waypoint = dict(waypoint_by_name[name])
        # planned_yaw e' usato solo come informazione di arrivo prevista. Nel
        # ciclo principale ricalcoliamo comunque base_yaw usando current_pose.
        waypoint["planned_yaw"] = yaw_between(current, waypoint)

        if index == 0:
            # Primo tratto: costo calcolato online dalla posa reale.
            waypoint["planned_cost_from_previous"] = start_costs[name]
        elif index - 1 < len(edge_costs):
            # Tratti successivi: costi precalcolati nel file route.
            waypoint["planned_cost_from_previous"] = edge_costs[index - 1]

        ordered.append(waypoint)
        current = {
            "x": waypoint["x"],
            "y": waypoint["y"],
            "yaw": waypoint["planned_yaw"],
        }

    return ordered


def target_found(target_state):
    """Dice se il detector ha gia pubblicato una posa ArUco valida."""
    return target_state is not None and target_state["found"]


def refresh_target_state(navigator, target_state):
    """Esegue una callback ROS pendente e poi controlla il flag target."""
    if target_state is None:
        return False

    # Questo spin_once e' cio che rende reattiva la cancellazione della search:
    # senza processare callback, /aruco/pose non aggiornerebbe target_state.
    rclpy.spin_once(navigator, timeout_sec=0.0)
    return target_found(target_state)


def normalize_costmap_value(value):
    """Converte un valore costmap in intero 0-255."""
    # Alcune rappresentazioni Python possono esporre uint8 come signed byte.
    # Riportiamo sempre il valore nel range usato da Nav2:
    # 0 libero, 253 inscribed, 254 lethal, 255 unknown.
    value = int(value)
    if value < 0:
        value += 256
    return value


def transform_xy(tf_buffer, x, y, source_frame, target_frame, timeout_sec):
    """Trasforma un punto 2D dal frame del waypoint al frame della costmap."""
    if not source_frame or source_frame == target_frame:
        return float(x), float(y)

    transform = tf_buffer.lookup_transform(
        target_frame,
        source_frame,
        Time(),
        timeout=Duration(seconds=float(timeout_sec)),
    )
    translation = transform.transform.translation
    yaw = quaternion_to_yaw(transform.transform.rotation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    # lookup_transform(target, source) restituisce la trasformazione che porta
    # coordinate espresse in source dentro target: p_target = R*p_source + t.
    transformed_x = translation.x + cos_yaw * float(x) - sin_yaw * float(y)
    transformed_y = translation.y + sin_yaw * float(x) + cos_yaw * float(y)
    return transformed_x, transformed_y


def request_costmap(navigator, costmap_client, timeout_sec):
    """Richiede una snapshot della local costmap senza bloccare troppo il nodo."""
    if not costmap_client.wait_for_service(timeout_sec=0.05):
        return None

    future = costmap_client.call_async(GetCostmap.Request())
    deadline = time.monotonic() + max(0.05, float(timeout_sec))

    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        # Continuiamo a processare callback ROS mentre aspettiamo la risposta
        # del servizio, cosi /aruco/pose e lo stato Nav2 non restano congelati.
        rclpy.spin_once(navigator, timeout_sec=0.02)

    if not future.done():
        return None

    result = future.result()
    if result is None:
        return None

    return result.map


def costmap_point_is_blocked(costmap, x, y, radius, threshold, unknown_is_blocked):
    """Controlla il costo della cella del waypoint in un piccolo intorno."""
    metadata = costmap.metadata
    resolution = float(metadata.resolution)
    if resolution <= 0.0:
        return False, None

    origin = metadata.origin.position
    mx = int((float(x) - float(origin.x)) / resolution)
    my = int((float(y) - float(origin.y)) / resolution)
    size_x = int(metadata.size_x)
    size_y = int(metadata.size_y)

    if mx < 0 or my < 0 or mx >= size_x or my >= size_y:
        # Il waypoint non e' ancora dentro la finestra locale: non possiamo
        # giudicarlo occupato, quindi lasciamo che Nav2 continui ad avvicinarsi.
        return False, None

    radius_cells = max(0, int(math.ceil(float(radius) / resolution)))
    max_cost = 0

    for cy in range(max(0, my - radius_cells), min(size_y, my + radius_cells + 1)):
        for cx in range(max(0, mx - radius_cells), min(size_x, mx + radius_cells + 1)):
            distance = math.hypot(cx - mx, cy - my) * resolution
            if distance > float(radius):
                continue

            index = cy * size_x + cx
            cell_cost = normalize_costmap_value(costmap.data[index])

            if cell_cost == 255 and not unknown_is_blocked:
                continue

            max_cost = max(max_cost, cell_cost)
            if cell_cost >= int(threshold):
                return True, cell_cost

    return False, max_cost


def waypoint_blocked_by_local_costmap(navigator, pose, label, local_cost_check):
    """Dice se il goal cade su una cella locale troppo costosa."""
    if not local_cost_check or not local_cost_check["enabled"]:
        return False

    costmap = request_costmap(
        navigator=navigator,
        costmap_client=local_cost_check["client"],
        timeout_sec=local_cost_check["service_timeout_sec"],
    )
    if costmap is None:
        navigator.get_logger().debug("Local costmap service not available yet")
        return False

    costmap_frame = costmap.header.frame_id
    waypoint_frame = pose.header.frame_id

    try:
        x, y = transform_xy(
            tf_buffer=local_cost_check["tf_buffer"],
            x=pose.pose.position.x,
            y=pose.pose.position.y,
            source_frame=waypoint_frame,
            target_frame=costmap_frame,
            timeout_sec=local_cost_check["tf_timeout_sec"],
        )
    except TransformException as error:
        navigator.get_logger().debug(
            f"Cannot transform {label} from {waypoint_frame} "
            f"to {costmap_frame}: {error}"
        )
        return False

    blocked, cost = costmap_point_is_blocked(
        costmap=costmap,
        x=x,
        y=y,
        radius=local_cost_check["radius"],
        threshold=local_cost_check["threshold"],
        unknown_is_blocked=local_cost_check["unknown_is_blocked"],
    )

    if blocked:
        navigator.get_logger().warn(
            f"Skipping {label}: local costmap cost near goal is {cost} "
            f"(threshold {local_cost_check['threshold']})"
        )
        return True

    return False


def navigate_to_pose(
    navigator,
    pose,
    label,
    target_state,
    timeout_sec=0.0,
    local_cost_check=None,
    behavior_tree="",
):
    """Invia un goal Nav2 e lo cancella se durante il moto troviamo l'ArUco."""
    if refresh_target_state(navigator, target_state):
        navigator.get_logger().info(f"Target already found before navigating to {label}")
        return "target_found"

    if waypoint_blocked_by_local_costmap(navigator, pose, label, local_cost_check):
        return "blocked"

    # goToPose usa NavigateToPose/BT Navigator: qui il robot si muove davvero
    # tra due waypoint della route.
    navigator.get_logger().info(f"Navigating to {label}")
    if behavior_tree:
        navigator.goToPose(pose, behavior_tree=behavior_tree)
    else:
        navigator.goToPose(pose)

    start_time = time.monotonic()
    last_cost_check_time = 0.0

    while not navigator.isTaskComplete():
        # Durante la navigazione continuiamo ad ascoltare /aruco/pose.
        if target_found(target_state):
            navigator.get_logger().info("Target found during navigation, canceling goal")
            navigator.cancelTask()
            return "target_found"

        now = time.monotonic()
        if (
            local_cost_check
            and local_cost_check["enabled"]
            and now - last_cost_check_time >= local_cost_check["check_period_sec"]
        ):
            last_cost_check_time = now
            if waypoint_blocked_by_local_costmap(navigator, pose, label, local_cost_check):
                navigator.get_logger().warn(
                    f"Canceling navigation to {label}: goal is occupied locally"
                )
                navigator.cancelTask()
                return "blocked"

        if timeout_sec > 0.0 and time.monotonic() - start_time > timeout_sec:
            navigator.get_logger().warn(f"Timeout while navigating to {label}")
            navigator.cancelTask()
            return "timeout"

        # Serve a far avanzare sia le callback del navigator sia la subscription
        # al topic ArUco, evitando un loop bloccante.
        rclpy.spin_once(navigator, timeout_sec=0.2)

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(f"Reached {label}")
        return "succeeded"

    if result == TaskResult.CANCELED:
        navigator.get_logger().warn(f"Navigation to {label} canceled")
        return "canceled"

    navigator.get_logger().error(f"Navigation to {label} failed")
    return "failed"


def execute_spin(navigator, spin_client, angle, label, target_state, timeout_sec=0.0):
    """Esegue una rotazione relativa Nav2 Spin continuando ad ascoltare ArUco."""
    # Prima di inviare una nuova action Spin processiamo eventuali callback gia
    # arrivate su /aruco/pose: se il target e' stato visto, non ha senso ruotare.
    if refresh_target_state(navigator, target_state):
        navigator.get_logger().info(f"Target already found before scan {label}")
        return "target_found"

    # Angoli praticamente nulli vengono considerati gia completati. Questo
    # evita di mandare action inutili al behavior_server.
    if abs(float(angle)) < 1e-6:
        navigator.get_logger().info(f"Skipping zero scan rotation for {label}")
        return "succeeded"

    # Lo Spin e' una action servita dal behavior_server di Nav2. Se il server
    # non e' disponibile, la scansione non puo essere eseguita.
    if not spin_client.wait_for_server(timeout_sec=2.0):
        navigator.get_logger().error("Spin action server is not available")
        return "failed"

    # goal_timeout_sec, se passato dal launch, diventa il tempo massimo
    # esplicito. Se resta 0, stimiamo un time_allowance coerente con i limiti
    # di velocita/accelerazione del behavior_server.
    allowance = float(timeout_sec) if timeout_sec > 0.0 else spin_time_allowance(angle)

    goal = Spin.Goal()
    # target_yaw della action Spin NON e' uno yaw assoluto in mappa: e' la
    # rotazione relativa da compiere a partire dall'orientamento attuale.
    # Il segno e' quindi fondamentale: positivo antiorario, negativo orario.
    goal.target_yaw = float(angle)
    # time_allowance non imposta la velocita dello Spin. Dice solo a Nav2 entro
    # quanto tempo massimo la rotazione deve concludersi prima di fallire.
    goal.time_allowance = duration_msg(allowance)

    navigator.get_logger().info(
        f"Scanning {label}: relative spin {float(angle):.2f} rad "
        f"({math.degrees(float(angle)):.1f} deg), "
        f"time allowance {allowance:.2f} s"
    )

    send_goal_future = spin_client.send_goal_async(goal)
    target_detected_before_accept = False
    while rclpy.ok() and not send_goal_future.done():
        # Anche mentre aspettiamo che il behavior_server accetti il goal,
        # continuiamo a far girare l'executor del nodo. Cosi una detection
        # arrivata in questo intervallo puo fermare subito la scansione.
        if target_found(target_state):
            target_detected_before_accept = True
        rclpy.spin_once(navigator, timeout_sec=0.05)

    goal_handle = send_goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        navigator.get_logger().error(f"Spin goal rejected for {label}")
        return "failed"

    if target_detected_before_accept or target_found(target_state):
        # Il goal potrebbe essere stato accettato proprio mentre arrivava
        # /aruco/pose. Lo cancelliamo per non lasciare una rotazione pendente.
        navigator.get_logger().info("Target found before scan started, canceling spin")
        cancel_future = goal_handle.cancel_goal_async()
        while rclpy.ok() and not cancel_future.done():
            rclpy.spin_once(navigator, timeout_sec=0.05)
        return "target_found"

    result_future = goal_handle.get_result_async()
    start_time = time.monotonic()

    while rclpy.ok() and not result_future.done():
        # Questa e' la parte reattiva della scansione: ad ogni tick lasciamo
        # entrare le callback ROS e, se ArUco e' stato visto, cancelliamo Spin.
        if target_found(target_state):
            navigator.get_logger().info("Target found during scan, canceling spin")
            cancel_future = goal_handle.cancel_goal_async()
            while rclpy.ok() and not cancel_future.done():
                rclpy.spin_once(navigator, timeout_sec=0.05)
            return "target_found"

        if timeout_sec > 0.0 and time.monotonic() - start_time > timeout_sec:
            # Questo timeout e' un watchdog lato nostro, separato dal
            # time_allowance passato alla action. Scatta solo se l'utente ha
            # configurato goal_timeout_sec > 0.
            navigator.get_logger().warn(f"Timeout while scanning {label}")
            cancel_future = goal_handle.cancel_goal_async()
            while rclpy.ok() and not cancel_future.done():
                rclpy.spin_once(navigator, timeout_sec=0.05)
            return "timeout"

        rclpy.spin_once(navigator, timeout_sec=0.05)

    result = result_future.result()
    # Nav2 restituisce lo stato action standard: succeeded/canceled/aborted.
    # Lo traduciamo nelle stringhe usate dal ciclo principale della missione.
    if result.status == GoalStatus.STATUS_SUCCEEDED:
        navigator.get_logger().info(f"Completed scan {label}")
        return "succeeded"

    if result.status == GoalStatus.STATUS_CANCELED:
        navigator.get_logger().warn(f"Scan {label} canceled")
        return "canceled"

    navigator.get_logger().error(f"Scan {label} failed with status {result.status}")
    return "failed"


def execute_scan(
    navigator,
    spin_client,
    waypoint,
    base_yaw,
    scan_angles,
    timeout_sec,
    target_state,
):
    """Esegue tutti i settori di scansione di un waypoint con Spin firmati."""
    # base_yaw e' lo yaw con cui il robot arriva al waypoint dopo la navigazione.
    # Gli scan_angles nel file YAML restano relativi a questo verso di arrivo.
    current_yaw = base_yaw

    for angle in scan_angles:
        requested_angle = float(angle)
        # Semantica mantenuta dal codice precedente: lo scan angle e' sommato
        # allo yaw base di arrivo. Quindi +1.57 significa "guarda 90 gradi a
        # sinistra rispetto al verso con cui sono arrivato al waypoint".
        target_yaw = normalize_angle(base_yaw + requested_angle)
        # Nav2 Spin vuole una rotazione relativa, non un target assoluto.
        # signed_rotation_to_target trasforma target_yaw in una rotazione
        # relativa da current_yaw, preservando il verso richiesto dal segno
        # dello scan angle nel file YAML.
        spin_angle = signed_rotation_to_target(
            current_yaw=current_yaw,
            target_yaw=target_yaw,
            requested_angle=requested_angle,
        )

        label = f"{waypoint['name']} scan angle {angle:.2f}"
        status = execute_spin(
            navigator,
            spin_client,
            spin_angle,
            label,
            target_state,
            timeout_sec,
        )

        # Dopo una rotazione completata assumiamo che lo yaw finale sia quello
        # appena richiesto. Questo serve per calcolare correttamente la
        # rotazione relativa del settore successivo.
        current_yaw = target_yaw

        if status == "target_found":
            return "target_found", current_yaw

        if status != "succeeded":
            return status, current_yaw

    return "succeeded", current_yaw


def scan_angles_for_waypoint(waypoint, default_scan_angles, previous_waypoint_name, navigator):
    """Legge gli scan angle e li inverte se il waypoint arriva dal lato opposto."""
    # Ogni waypoint puo definire i propri scan_angles. Se non li definisce,
    # usiamo i default della missione.
    scan_angles = waypoint.get("scan_angles", default_scan_angles)
    if scan_angles is None:
        scan_angles = default_scan_angles

    # Convertiamo a float subito per evitare differenze tra numeri letti come
    # int, float o stringhe nel file YAML.
    scan_angles = [float(angle) for angle in scan_angles]
    # invert_scan_angles_from contiene i nomi dei waypoint precedenti per cui
    # il settore nascosto e' speculare. In quel caso basta cambiare segno agli
    # angoli: positivo <-> negativo, quindi antiorario <-> orario.
    invert_from = waypoint.get("invert_scan_angles_from", [])
    if invert_from is None:
        invert_from = []

    invert_from = {str(name) for name in invert_from}
    if previous_waypoint_name is not None and str(previous_waypoint_name) in invert_from:
        inverted = [-angle for angle in scan_angles]
        navigator.get_logger().info(
            f"Inverting scan angles for {waypoint['name']} "
            f"because previous waypoint is {previous_waypoint_name}: {inverted}"
        )
        return inverted

    return scan_angles


def main():
    """Avvia il nodo ROS della missione di ricerca."""
    rclpy.init()

    # BasicNavigator e' sia il client verso Nav2 sia un nodo rclpy: per questo
    # puo dichiarare parametri, creare subscription e inviare goal di movimento.
    navigator = BasicNavigator()
    target_state = None
    target_subscription = None

    # Parametri del nodo. Vengono valorizzati dal launch file, ma restano
    # modificabili da terminale se vogliamo testare missioni o route diverse.
    navigator.declare_parameter("mission_file", "")
    navigator.declare_parameter("route_file", "")
    navigator.declare_parameter("cost_matrix_file", "")
    navigator.declare_parameter("visited_waypoints_file", "")
    navigator.declare_parameter("planner_id", "")
    navigator.declare_parameter("behavior_tree", "")
    navigator.declare_parameter("initial_pose_topic", "initialpose")
    navigator.declare_parameter("target_pose_topic", "/aruco/pose")
    navigator.declare_parameter("goal_timeout_sec", 0.0)
    navigator.declare_parameter("continue_on_failure", True)
    navigator.declare_parameter("waypoint_local_cost_check_enabled", True)
    navigator.declare_parameter("waypoint_local_costmap_service", "/local_costmap/get_costmap")
    navigator.declare_parameter("waypoint_local_cost_threshold", 253)
    navigator.declare_parameter("waypoint_local_cost_radius", 0.05)
    navigator.declare_parameter("waypoint_local_cost_unknown_is_blocked", False)
    navigator.declare_parameter("waypoint_local_cost_check_period_sec", 0.75)
    navigator.declare_parameter("waypoint_local_cost_service_timeout_sec", 0.20)
    navigator.declare_parameter("waypoint_local_cost_tf_timeout_sec", 0.10)

    mission_file = navigator.get_parameter("mission_file").value
    route_file = navigator.get_parameter("route_file").value
    legacy_cost_matrix_file = navigator.get_parameter("cost_matrix_file").value
    visited_waypoints_file = navigator.get_parameter("visited_waypoints_file").value
    planner_id = navigator.get_parameter("planner_id").value
    behavior_tree = navigator.get_parameter("behavior_tree").value
    initial_pose_topic = navigator.get_parameter("initial_pose_topic").value
    target_pose_topic = navigator.get_parameter("target_pose_topic").value
    goal_timeout_sec = navigator.get_parameter("goal_timeout_sec").value
    continue_on_failure = navigator.get_parameter("continue_on_failure").value
    waypoint_local_cost_check_enabled = navigator.get_parameter(
        "waypoint_local_cost_check_enabled"
    ).value
    waypoint_local_costmap_service = navigator.get_parameter(
        "waypoint_local_costmap_service"
    ).value
    waypoint_local_cost_threshold = navigator.get_parameter(
        "waypoint_local_cost_threshold"
    ).value
    waypoint_local_cost_radius = navigator.get_parameter(
        "waypoint_local_cost_radius"
    ).value
    waypoint_local_cost_unknown_is_blocked = navigator.get_parameter(
        "waypoint_local_cost_unknown_is_blocked"
    ).value
    waypoint_local_cost_check_period_sec = navigator.get_parameter(
        "waypoint_local_cost_check_period_sec"
    ).value
    waypoint_local_cost_service_timeout_sec = navigator.get_parameter(
        "waypoint_local_cost_service_timeout_sec"
    ).value
    waypoint_local_cost_tf_timeout_sec = navigator.get_parameter(
        "waypoint_local_cost_tf_timeout_sec"
    ).value

    local_cost_check = None
    if waypoint_local_cost_check_enabled:
        # La local costmap e' in genere in frame odom, mentre i waypoint sono
        # in map. Per controllare il costo locale del goal serve quindi TF.
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, navigator)
        local_cost_check = {
            "enabled": True,
            "client": navigator.create_client(
                GetCostmap,
                str(waypoint_local_costmap_service),
            ),
            "tf_buffer": tf_buffer,
            "tf_listener": tf_listener,
            "threshold": int(waypoint_local_cost_threshold),
            "radius": float(waypoint_local_cost_radius),
            "unknown_is_blocked": bool(waypoint_local_cost_unknown_is_blocked),
            "check_period_sec": float(waypoint_local_cost_check_period_sec),
            "service_timeout_sec": float(waypoint_local_cost_service_timeout_sec),
            "tf_timeout_sec": float(waypoint_local_cost_tf_timeout_sec),
        }
        navigator.get_logger().info(
            "Local waypoint cost check enabled: "
            f"service={waypoint_local_costmap_service}, "
            f"threshold={int(waypoint_local_cost_threshold)}, "
            f"radius={float(waypoint_local_cost_radius):.2f} m"
        )

    # La missione YAML e' obbligatoria: contiene frame, waypoint, tipi
    # transit/scan e settori angolari da coprire.
    if not mission_file:
        navigator.get_logger().error("Parameter mission_file is required")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    # La subscription ad ArUco viene creata subito, prima di caricare route e
    # muovere il robot, cosi qualunque detection successiva puo interrompere la
    # ricerca nel modo piu rapido possibile.
    target_state, target_subscription = create_target_subscription(
        navigator,
        target_pose_topic,
    )

    # Il file missione definisce frame della mappa, coordinate dei waypoint e
    # settori di scan per i waypoint di tipo scan.
    mission = load_mission(mission_file)

    frame_id = mission.get("frame_id", "map")
    defaults = mission.get("defaults", {})
    default_scan_angles = defaults.get("scan_angles", [0.0, 1.57, 3.14, -1.57])
    first_waypoint_policy = first_waypoint_cost_policy(mission_file)
    navigator.get_logger().info(
        f"First waypoint selection policy: {first_waypoint_policy}"
    )

    visited_waypoints_file_was_provided = bool(str(visited_waypoints_file).strip())
    visit_state_file = normalize_state_file_path(visited_waypoints_file)
    resume_state = None
    resume_from_existing_state = False
    if (
        visited_waypoints_file_was_provided
        and visit_state_file
        and os.path.exists(visit_state_file)
    ):
        try:
            resume_state = load_visit_state(visit_state_file)
            resume_from_existing_state = True
            navigator.get_logger().info(
                f"Resuming waypoint mission from visit state: {visit_state_file}"
            )
        except Exception as error:
            navigator.get_logger().error(
                f"Could not load visited waypoints file {visit_state_file}: {error}"
            )
            navigator.destroy_subscription(target_subscription)
            navigator.destroy_node()
            rclpy.shutdown()
            return
    elif visited_waypoints_file_was_provided and visit_state_file:
        navigator.get_logger().info(
            f"Visited waypoints file does not exist yet; it will be created: "
            f"{visit_state_file}"
        )

    # La posa iniziale arriva da RViz come PoseWithCovarianceStamped:
    # - per AMCL/Nav2 la conserviamo come PoseStamped con quaternion;
    # - per la route selection la convertiamo anche in x/y/yaw.
    start_pose, initial_pose_stamped = wait_for_initial_pose(
        navigator,
        initial_pose_topic,
        frame_id,
    )
    if start_pose is None:
        navigator.get_logger().error("Initial pose was not received")
        navigator.destroy_subscription(target_subscription)
        navigator.destroy_node()
        rclpy.shutdown()
        return

    navigator.get_logger().info(
        f"Initial pose received: x={start_pose['x']:.2f}, "
        f"y={start_pose['y']:.2f}, yaw={start_pose['yaw']:.2f}"
    )

    navigator.setInitialPose(initial_pose_stamped)
    navigator.get_logger().info("Initial pose copied into BasicNavigator")

    # Da qui in poi Nav2 deve essere attivo: planner_server serve per scegliere
    # il primo waypoint, bt_navigator/controller_server per muovere il robot.
    navigator.get_logger().info("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()

    # Spin viene eseguito dal behavior_server. Lo teniamo come action separata
    # per coprire settori angolari con verso controllato durante gli scan.
    spin_client = ActionClient(navigator, Spin, "spin")
    if not spin_client.wait_for_server(timeout_sec=5.0):
        navigator.get_logger().warn(
            "Spin action server is not available yet; scan waypoints may fail"
        )

    # Compatibilita con vecchi launch: prima questo parametro si chiamava
    # cost_matrix_file. Ora deve puntare al file route offline.
    if not route_file and legacy_cost_matrix_file:
        route_file = legacy_cost_matrix_file
        navigator.get_logger().warn(
            "Parameter cost_matrix_file is deprecated; use route_file instead"
        )

    if not route_file and not resume_from_existing_state:
        navigator.get_logger().error("Parameter route_file is required")
        navigator.destroy_subscription(target_subscription)
        navigator.destroy_node()
        rclpy.shutdown()
        return

    visit_state = resume_state
    if resume_from_existing_state:
        if not planner_id:
            planner_id = visit_state.get("planner_id", "")
            navigator.get_logger().info(
                f"Using planner_id from visit state: '{planner_id}'"
            )

        try:
            ordered_waypoints = build_ordered_waypoints_from_saved_route(
                mission=mission,
                state=visit_state,
            )
        except Exception as error:
            navigator.get_logger().error(f"Could not resume saved route: {error}")
            navigator.destroy_subscription(target_subscription)
            navigator.destroy_node()
            rclpy.shutdown()
            return

        navigator.get_logger().info(
            f"Resume state contains {len(processed_waypoint_names(visit_state))} "
            f"already processed waypoint(s); "
            f"{len(ordered_waypoints)} waypoint(s) remain"
        )
    else:
        navigator.get_logger().info(f"Loading offline route file: {route_file}")
        try:
            # Il file route contiene una route ottimizzata per ogni possibile
            # primo waypoint. Qui lo leggiamo e recuperiamo anche il planner_id.
            route_data = load_route_file(route_file)
            route_planner_id = route_data.get("planner_id", "")

            if not planner_id:
                # Se il launch non forza un planner, usiamo quello con cui sono
                # state calcolate le route offline.
                planner_id = route_planner_id
                navigator.get_logger().info(
                    f"Using planner_id from route file: '{planner_id}'"
                )
            elif route_planner_id and planner_id != route_planner_id:
                # Planner offline e planner online diversi possono dare percorsi
                # non coerenti. Non blocchiamo, ma lo rendiamo evidente nei log.
                navigator.get_logger().warn(
                    f"planner_id parameter '{planner_id}' differs from "
                    f"route file planner_id '{route_planner_id}'"
                )

            # Online calcoliamo solo i path dalla posa iniziale reale ai possibili
            # primi waypoint. Tutto l'ordine successivo arriva dal MILP offline.
            ordered_waypoints = order_waypoints_from_routes(
                navigator=navigator,
                frame_id=frame_id,
                start_pose=start_pose,
                waypoints=mission["waypoints"],
                route_data=route_data,
                planner_id=planner_id,
                first_waypoint_policy=first_waypoint_policy,
            )
        except Exception as error:
            navigator.get_logger().error(f"Offline route ordering failed: {error}")
            navigator.destroy_subscription(target_subscription)
            navigator.destroy_node()
            rclpy.shutdown()
            return

        if not visit_state_file:
            visit_state_file = default_visit_state_file()

        visit_state = make_visit_state(
            mission_file=mission_file,
            route_file=route_file,
            planner_id=planner_id,
            frame_id=frame_id,
            ordered_waypoints=ordered_waypoints,
        )
        try:
            write_visit_state(visit_state_file, visit_state)
            navigator.get_logger().info(
                f"Created waypoint visit state file: {visit_state_file}"
            )
        except Exception as error:
            navigator.get_logger().error(
                f"Could not create waypoint visit state file {visit_state_file}: {error}"
            )
            navigator.destroy_subscription(target_subscription)
            navigator.destroy_node()
            rclpy.shutdown()
            return

    if refresh_target_state(navigator, target_state):
        navigator.get_logger().info("Target found before starting waypoint navigation")
        navigator.destroy_subscription(target_subscription)
        navigator.destroy_node()
        rclpy.shutdown()
        return

    navigator.get_logger().info("Waypoint visit order from initial pose:")
    for index, waypoint in enumerate(ordered_waypoints):
        # Stampiamo la route risolta prima di muoverci: sul robot reale e' utile
        # per capire subito quale giro e' stato scelto.
        navigator.get_logger().info(
            f"{index + 1}. {waypoint['name']} "
            f"type={waypoint.get('type', 'transit')} "
            f"x={float(waypoint['x']):.2f} y={float(waypoint['y']):.2f} "
            f"cost_from_previous="
            f"{float(waypoint.get('planned_cost_from_previous', 0.0)):.2f}"
        )

    # current_pose rappresenta il punto da cui parte il prossimo tratto.
    # All'inizio coincide con la posa RViz; dopo ogni waypoint contiene anche
    # il nome del waypoint precedente, usato per invert_scan_angles_from.
    current_pose = dict(start_pose)
    if resume_from_existing_state and visit_state.get("last_reached_waypoint"):
        current_pose["name"] = str(visit_state["last_reached_waypoint"])

    for waypoint in ordered_waypoints:
        # Prima di inviare ogni nuovo goal controlliamo ArUco: se il target e'
        # gia stato visto, interrompiamo la ricerca e lasciamo spazio al follow.
        if refresh_target_state(navigator, target_state):
            navigator.get_logger().info("Target found before sending next waypoint")
            break

        waypoint_type = waypoint.get("type", "transit")
        previous_waypoint_name = current_pose.get("name")
        # base_yaw e' la direzione del tratto current_pose -> waypoint. La
        # usiamo come orientamento finale del goal e come riferimento per gli
        # scan_angles relativi definiti nello YAML.
        base_yaw = yaw_between(current_pose, waypoint)

        # make_pose converte x/y/yaw in PoseStamped. Qui serve yaw -> quaternion
        # perche Nav2 accetta goal geometry_msgs/Pose, non angoli scalari.
        goal_pose = make_pose(
            navigator=navigator,
            frame_id=frame_id,
            x=waypoint["x"],
            y=waypoint["y"],
            yaw=base_yaw,
        )

        status = navigate_to_pose(
            navigator=navigator,
            pose=goal_pose,
            label=waypoint["name"],
            target_state=target_state,
            timeout_sec=goal_timeout_sec,
            local_cost_check=local_cost_check,
            behavior_tree=behavior_tree,
        )

        if status == "target_found":
            break

        if status != "succeeded":
            navigator.get_logger().warn(
                f"Moving to next waypoint after {waypoint['name']} status: {status}"
            )
            record_waypoint_state(
                state_file=visit_state_file,
                state=visit_state,
                waypoint=waypoint,
                status=status,
                reached=False,
                navigator=navigator,
            )
            continue

        current_pose = {
            "name": waypoint_name(waypoint),
            "x": waypoint["x"],
            "y": waypoint["y"],
            "yaw": base_yaw,
        }
        record_waypoint_state(
            state_file=visit_state_file,
            state=visit_state,
            waypoint=waypoint,
            status=status,
            reached=True,
            navigator=navigator,
        )

        # I waypoint transit sono solo nodi di passaggio del grafo: non fanno
        # ruotare il robot e non eseguono sweep della camera.
        if waypoint_type == "transit":
            navigator.get_logger().info(f"{waypoint['name']} is transit, moving on")
            continue

        if waypoint_type == "scan":
            # I waypoint scan coprono settori angolari firmati con Nav2 Spin.
            # La callback ArUco resta attiva durante tutta la rotazione.
            scan_angles = scan_angles_for_waypoint(
                waypoint=waypoint,
                default_scan_angles=default_scan_angles,
                previous_waypoint_name=previous_waypoint_name,
                navigator=navigator,
            )

            status, _final_yaw = execute_scan(
                navigator=navigator,
                spin_client=spin_client,
                waypoint=waypoint,
                base_yaw=base_yaw,
                scan_angles=scan_angles,
                timeout_sec=goal_timeout_sec,
                target_state=target_state,
            )

            # _final_yaw e' lo yaw finale stimato dopo l'ultimo Spin. Lo
            # salviamo perche il prossimo tratto deve partire dall'orientamento
            # effettivamente raggiunto dopo lo scan.
            current_pose["yaw"] = _final_yaw

            if status == "target_found":
                break

            if status != "succeeded" and not continue_on_failure:
                break

            continue

        navigator.get_logger().warn(
            f"Unknown waypoint type: {waypoint_type}; expected 'transit' or 'scan'"
        )

    navigator.get_logger().info("Search mission finished")
    if target_subscription is not None:
        navigator.destroy_subscription(target_subscription)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
