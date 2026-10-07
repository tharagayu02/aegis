import json
import logging
import math
import os
import time
from datetime import datetime
from urllib.error import URLError
from urllib.request import Request, urlopen

from dotenv import load_dotenv
from sqlalchemy import desc, select

from backend.app.database import BACKEND_DIR, SessionLocal
from backend.app.geo import (
    distance_to_threat_m,
    route_distance_m,
    segment_intersects_threat,
)
from backend.app.models import Alert, Drone, FlightPath, Telemetry, Threat
from backend.path_planning import plan_astar_route

load_dotenv(BACKEND_DIR / ".env")
logging.basicConfig(
    level=os.getenv("SIMULATOR_LOG_LEVEL", "INFO"),
    format="%(asctime)s %(levelname)s %(message)s",
)
logger = logging.getLogger("drone-simulator")

DRONE_IDS = ("RECON-01", "RECON-02", "RECON-03", "RECON-04")
API_URL = os.getenv("API_BASE_URL", "http://127.0.0.1:8000").rstrip("/")
INTERVAL_SECONDS = max(float(os.getenv("SIMULATOR_INTERVAL_SECONDS", "1")), 0.25)
SPEED_METERS_PER_SECOND = 32.0
PATROL_HEADINGS = {"RECON-01": 85.0, "RECON-02": 205.0, "RECON-03": 25.0, "RECON-04": 300.0}
DEMO_DRONE = "RECON-01"


def request_json(path: str, body: dict | None = None):
    data = json.dumps(body).encode("utf-8") if body is not None else None
    request = Request(
        f"{API_URL}{path}",
        data=data,
        headers={"Content-Type": "application/json"} if data else {},
        method="POST" if data else "GET",
    )
    try:
        with urlopen(request, timeout=4) as response:
            return json.loads(response.read())
    except (URLError, TimeoutError) as error:
        logger.warning("Could not reach API at %s: %s", path, error)
        return None


def advance(
    start: tuple[float, float], destination: tuple[float, float], distance_m: float
) -> tuple[float, float]:
    latitude, longitude = start
    target_latitude, target_longitude = destination
    north = (target_latitude - latitude) * 110540
    east = (
        (target_longitude - longitude)
        * 111320
        * math.cos(math.radians(latitude))
    )
    distance = math.hypot(north, east)
    if distance <= distance_m or distance < 0.01:
        return destination
    fraction = distance_m / distance
    return (
        latitude + north * fraction / 110540,
        longitude
        + east
        * fraction
        / (111320 * max(math.cos(math.radians(latitude)), 0.01)),
    )


def advance_route(route: list[dict], route_index: int, current: tuple[float, float]):
    remaining = SPEED_METERS_PER_SECOND
    while route_index < len(route):
        target = (route[route_index]["latitude"], route[route_index]["longitude"])
        north = (target[0] - current[0]) * 110540
        east = (
            (target[1] - current[1])
            * 111320
            * math.cos(math.radians(current[0]))
        )
        distance = math.hypot(north, east)
        if distance <= remaining:
            current = target
            route_index += 1
            remaining -= distance
            if remaining <= 0:
                break
        else:
            current = advance(current, target, remaining)
            break
    return current, route_index


def get_position(db, drone: Drone) -> tuple[float, float]:
    latest = db.scalar(
        select(Telemetry)
        .where(Telemetry.drone_id == drone.drone_id)
        .order_by(desc(Telemetry.timestamp), desc(Telemetry.id))
        .limit(1)
    )
    if latest:
        return float(latest.latitude), float(latest.longitude)
    return float(drone.latitude), float(drone.longitude)


def active_threats(db) -> list[Threat]:
    return db.scalars(select(Threat).where(Threat.is_active.is_(True))).all()


def line_hits_threat(
    start: tuple[float, float], end: tuple[float, float], threats: list[Threat]
) -> Threat | None:
    return next(
        (
            threat
            for threat in threats
            if segment_intersects_threat(start, end, threat)
        ),
        None,
    )


def threat_polygon(threat: Threat) -> list[tuple[float, float]]:
    return [
        (float(threat.min_longitude), float(threat.min_latitude)),
        (float(threat.max_longitude), float(threat.min_latitude)),
        (float(threat.max_longitude), float(threat.max_latitude)),
        (float(threat.min_longitude), float(threat.max_latitude)),
        (float(threat.min_longitude), float(threat.min_latitude)),
    ]


def route_goal(start: tuple[float, float], threat: Threat) -> tuple[float, float]:
    center_lat = (float(threat.min_latitude) + float(threat.max_latitude)) / 2
    center_lon = (float(threat.min_longitude) + float(threat.max_longitude)) / 2
    north = (center_lat - start[0]) * 110540
    east = (
        (center_lon - start[1])
        * 111320
        * math.cos(math.radians(start[0]))
    )
    magnitude = max(math.hypot(north, east), 1)
    half_diagonal = math.hypot(
        (float(threat.max_latitude) - float(threat.min_latitude)) * 110540 / 2,
        (float(threat.max_longitude) - float(threat.min_longitude))
        * 111320
        * math.cos(math.radians(center_lat))
        / 2,
    )
    clearance = half_diagonal + 160
    return (
        center_lat + north / magnitude * clearance / 110540,
        center_lon
        + east
        / magnitude
        * clearance
        / (111320 * max(math.cos(math.radians(center_lat)), 0.01)),
    )


def plan_route(
    start: tuple[float, float], target: Threat, threats: list[Threat]
) -> tuple[tuple[float, float], list[dict[str, float]]]:
    goal = route_goal(start, target)
    route_points = plan_astar_route(
        start, goal, [threat_polygon(threat) for threat in threats]
    )
    route = [{"latitude": lat, "longitude": lon} for lat, lon in route_points]
    if len(route) < 2 or line_hits_threats(route, threats):
        raise RuntimeError(
            f"A* did not produce a route clear of active threat zones for {target.threat_id}"
        )
    return goal, route


def line_hits_threats(route: list[dict], threats: list[Threat]) -> bool:
    return any(
        line_hits_threat(
            (start["latitude"], start["longitude"]),
            (end["latitude"], end["longitude"]),
            threats,
        )
        is not None
        for start, end in zip(route, route[1:])
    )


def add_alert(db, drone_id: str, alert_type: str, message: str, severity: str) -> None:
    db.add(
        Alert(
            drone_id=drone_id,
            alert_type=alert_type,
            message=message,
            severity=severity,
            is_resolved=False,
        )
    )
    logger.warning("%s: %s", alert_type, message)


def save_route(
    db,
    drone_id: str,
    start: tuple[float, float],
    destination: tuple[float, float],
    route: list[dict],
) -> FlightPath:
    flight_path = FlightPath(
        drone_id=drone_id,
        path_type="A_STAR_AVOIDANCE",
        distance=route_distance_m(route),
        is_safe=True,
        route_json={
            "start": {"latitude": start[0], "longitude": start[1]},
            "destination": {
                "latitude": destination[0],
                "longitude": destination[1],
            },
            "path": route,
            "route_label": "OPTIMIZED A* ROUTE",
        },
    )
    db.add(flight_path)
    return flight_path


def choose_demo_drone(
    positions: dict[str, tuple[float, float]], threats: list[Threat]
) -> tuple[str, Threat] | None:
    candidates: list[tuple[float, str, Threat]] = []
    for drone_id, position in positions.items():
        for threat in threats:
            distance = distance_to_threat_m(*position, threat)
            if distance > 15:
                candidates.append((distance, drone_id, threat))
    if not candidates:
        return None
    _, drone_id, threat = min(candidates)
    return drone_id, threat


def run():
    with SessionLocal() as db:
        drones = {
            drone.drone_id: drone
            for drone in db.scalars(select(Drone).where(Drone.drone_id.in_(DRONE_IDS))).all()
        }
        missing_drones = set(DRONE_IDS) - set(drones)
        if missing_drones:
            raise RuntimeError(
                "Drone records are missing from MySQL. Run `python -m backend.init_db` first; "
                f"missing: {', '.join(sorted(missing_drones))}"
            )
        positions = {drone_id: get_position(db, drone) for drone_id, drone in drones.items()}
        batteries = {drone_id: float(drone.battery_pct) for drone_id, drone in drones.items()}

    routes: dict[str, dict] = {}
    drone_states: dict[str, dict] = {
        drone_id: {
            "status": "ACTIVE",
            "phase": "PATROL",
            "route": [],
            "route_label": None,
            "threat_id": None,
        }
        for drone_id in DRONE_IDS
    }
    demo_completed: set[str] = set()
    demo_was_enabled = False
    demo_drone_id = DEMO_DRONE
    tick = 0
    logger.info("MySQL simulator running; update interval %.2fs", INTERVAL_SECONDS)

    while True:
        tick_started = time.monotonic()
        simulation = request_json("/api/simulation/status") or {}
        demo_enabled = bool(simulation.get("demo_mode"))
        if demo_enabled and not demo_was_enabled:
            demo_completed.clear()
            with SessionLocal() as db:
                selection = choose_demo_drone(positions, active_threats(db))
            if selection:
                demo_drone_id = selection[0]
            logger.info("Demo mode enabled for %s", demo_drone_id)
        demo_was_enabled = demo_enabled

        with SessionLocal() as db:
            threats = active_threats(db)
            for drone_id in DRONE_IDS:
                drone = db.scalar(select(Drone).where(Drone.drone_id == drone_id))
                if drone is None:
                    logger.error("Drone %s disappeared from MySQL", drone_id)
                    continue
                current = positions[drone_id]
                state = drone_states[drone_id]
                batteries[drone_id] = max(0, batteries[drone_id] - 0.001)
                proposed: tuple[float, float]
                active_route = routes.get(drone_id)

                if active_route:
                    proposed, next_index = advance_route(
                        active_route["route"], active_route["index"], current
                    )
                    active_route["index"] = next_index
                    if next_index >= len(active_route["route"]):
                        state.update(
                            status="ACTIVE",
                            phase="COURSE CORRECTED · SAFE ROUTE ACTIVE",
                        )
                        add_alert(
                            db,
                            drone_id,
                            "SAFE_ROUTE",
                            f"{drone_id} course corrected; safe route active",
                            "LOW",
                        )
                        routes.pop(drone_id, None)
                        demo_completed.add(active_route["threat_id"])
                    else:
                        state.update(
                            status="WARNING",
                            phase="WARNING · RESTRICTED AREA · A* ROUTE CALCULATED · COURSE CORRECTED",
                        )
                else:
                    target_threat = None
                    if demo_enabled and drone_id == demo_drone_id:
                        possible = [
                            threat
                            for threat in threats
                            if threat.threat_id not in demo_completed
                        ]
                        if possible:
                            target_threat = min(
                                possible,
                                key=lambda threat: distance_to_threat_m(
                                    *current, threat
                                ),
                            )
                    if target_threat:
                        destination = (
                            (float(target_threat.min_latitude) + float(target_threat.max_latitude)) / 2,
                            (float(target_threat.min_longitude) + float(target_threat.max_longitude)) / 2,
                        )
                        proposed = advance(current, destination, SPEED_METERS_PER_SECOND)
                    else:
                        heading = math.radians(
                            PATROL_HEADINGS[drone_id]
                            + 12 * math.sin(tick / 24 + DRONE_IDS.index(drone_id))
                        )
                        proposed = (
                            current[0]
                            + math.cos(heading)
                            * SPEED_METERS_PER_SECOND
                            / 110540,
                            current[1]
                            + math.sin(heading)
                            * SPEED_METERS_PER_SECOND
                            / (111320 * max(math.cos(math.radians(current[0])), 0.01)),
                        )

                    blocked = line_hits_threat(current, proposed, threats)
                    if blocked is None and target_threat is not None:
                        if distance_to_threat_m(*proposed, target_threat) <= SPEED_METERS_PER_SECOND * 2:
                            blocked = target_threat

                    if blocked:
                        proposed = current
                        state.update(
                            status="WARNING",
                            phase="THREAT DETECTED · ROUTE BLOCKED · A* ROUTE CALCULATING",
                            threat_id=blocked.threat_id,
                        )
                        add_alert(
                            db,
                            drone_id,
                            "THREAT_DETECTED",
                            f"{drone_id} threat detected near {blocked.threat_id}",
                            "WARNING",
                        )
                        add_alert(
                            db,
                            drone_id,
                            "ROUTE_BLOCKED",
                            f"{drone_id} proposed route blocked by {blocked.threat_id}",
                            "WARNING",
                        )
                        try:
                            destination, route = plan_route(current, blocked, threats)
                        except RuntimeError as error:
                            logger.exception("A* route planning failed for %s", drone_id)
                            state.update(
                                phase="THREAT DETECTED · ROUTE BLOCKED · A* ROUTE FAILED",
                                route=[],
                                route_label=None,
                            )
                            add_alert(
                                db,
                                drone_id,
                                "A_STAR_ROUTE_FAILED",
                                str(error),
                                "CRITICAL",
                            )
                            drone.status = "WARNING"
                            drone.speed = 0
                            drone.latitude, drone.longitude = current
                            db.add(
                                Telemetry(
                                    drone_id=drone_id,
                                    timestamp=datetime.utcnow(),
                                    status="WARNING",
                                    battery_pct=batteries[drone_id],
                                    speed=0,
                                    latitude=current[0],
                                    longitude=current[1],
                                )
                            )
                            continue
                        save_route(db, drone_id, current, destination, route)
                        routes[drone_id] = {
                            "route": route,
                            "index": 1,
                            "threat_id": blocked.threat_id,
                        }
                        state.update(
                            status="WARNING",
                            phase="WARNING · RESTRICTED AREA · A* ROUTE CALCULATED · COURSE CORRECTED",
                            route=route,
                            route_label="OPTIMIZED A* ROUTE",
                            threat_id=blocked.threat_id,
                        )
                        add_alert(
                            db,
                            drone_id,
                            "A_STAR_ROUTE",
                            f"A* generated a safe route around {blocked.threat_id}",
                            "WARNING",
                        )
                        add_alert(
                            db,
                            drone_id,
                            "COURSE_CORRECTED",
                            f"{drone_id} course corrected onto the optimized route",
                            "WARNING",
                        )
                    elif state["status"] == "WARNING":
                        state.update(status="ACTIVE", phase="PATROL")
                        state.update(route=[], route_label=None, threat_id=None)

                if line_hits_threat(current, proposed, threats):
                    proposed = current
                    state.update(
                        status="WARNING",
                        phase="WARNING · RESTRICTED AREA · UNSAFE MOVEMENT REJECTED",
                    )
                positions[drone_id] = proposed
                drone.latitude = proposed[0]
                drone.longitude = proposed[1]
                drone.status = state["status"]
                drone.speed = SPEED_METERS_PER_SECOND
                drone.battery_pct = batteries[drone_id]
                db.add(
                    Telemetry(
                        drone_id=drone_id,
                        timestamp=datetime.utcnow(),
                        status=state["status"],
                        battery_pct=batteries[drone_id],
                        speed=SPEED_METERS_PER_SECOND,
                        latitude=proposed[0],
                        longitude=proposed[1],
                    )
                )
            try:
                db.commit()
            except Exception:
                db.rollback()
                logger.exception("Failed to commit simulator tick to MySQL")
                time.sleep(INTERVAL_SECONDS)
                continue

        request_json(
            "/api/simulation/state",
            {
                "live": True,
                "update_interval_seconds": INTERVAL_SECONDS,
                "demo_mode": demo_enabled,
                "demo_drone_id": demo_drone_id if demo_enabled else None,
                "drones": drone_states,
            },
        )
        tick += 1
        time.sleep(max(0, INTERVAL_SECONDS - (time.monotonic() - tick_started)))


if __name__ == "__main__":
    run()
